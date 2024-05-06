/* includes //{ */

#include <string.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <nlohmann/json.hpp>
#include <httplib/httplib.h>

#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <mrs_msgs/UavStatus.h>

//}

namespace iroc_bridge
{

using json = nlohmann::json;

/* class IROCBridge //{ */

class IROCBridge : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  ros::Time last_update_time_;

  std::thread th_http_srv_;
  httplib::Server http_srv_;
  std::unique_ptr<httplib::Client> http_client_;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_hw_api_gnss_;

  ros::ServiceClient sc_arm_;
  ros::ServiceClient sc_offboard_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | ------------------ Additional functions ------------------ |

  void parseLocalPosition(const mrs_msgs::UavStatusConstPtr& uav_status);
  void parseGlobalPosition(const sensor_msgs::NavSatFixConstPtr& global_position);
  void sendJsonMessage(const std::string& msg_type, const json& json_msg);

  struct svc_call_res_t
  {
    bool call_success;
    std::string message;
  };

  template <typename Svc_T>
  svc_call_res_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  svc_call_res_t callService(ros::ServiceClient& sc);

  svc_call_res_t callService(ros::ServiceClient& sc, const bool val);

  std::thread th_death_check_;
  void routine_death_check();
};
//}

/* onInit() //{ */

void IROCBridge::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();
  
  last_update_time_ = ros::Time(0);

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROCBridge");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");
  const auto url = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");

  if (!param_loader.loadedSuccessfully())
  {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  sc_arm_ = nh_.serviceClient<std_srvs::SetBool>("arm");
  sc_offboard_ = nh_.serviceClient<std_srvs::Trigger>("offboard");

  http_srv_.Get("/takeoff", [&](const httplib::Request &, httplib::Response &res)
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "Calling takeoff.");
        // firstly, arm the vehicle
        {
          const auto resp = callService(sc_arm_, true);
          if (!resp.call_success)
          {
            res.set_content(resp.message, "text/plain");
            return;
          }
        }

        // then, switch to offboard
        {
          const auto resp = callService<std_srvs::Trigger>(sc_offboard_);
          if (!resp.call_success)
          {
            res.set_content(resp.message, "text/plain");
            return;
          }
        }
        res.set_content("Taking off.", "text/plain");
      });

  th_http_srv_ = std::thread([&]()
      {
        http_srv_.listen(url, server_port);
      });
  th_http_srv_.detach();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "IROCBridge";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");
  sh_hw_api_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "hw_api_gnss_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &IROCBridge::timerMain, this);
  th_death_check_ = std::thread(&IROCBridge::routine_death_check, this);
  th_death_check_.detach();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROCBridge]: initialized");
  ROS_INFO("[IROCBridge]: --------------------");
  is_initialized_ = true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROCBridge::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  bool got_uav_status = sh_uav_status_.newMsg();
  bool got_hw_api_gnss = sh_hw_api_gnss_.newMsg();
  ros::Time time_now = ros::Time::now();

  if (!got_uav_status && !got_hw_api_gnss) {
    ros::Duration last_message_diff = time_now - last_update_time_;
    if(last_message_diff > ros::Duration(5.0)){
      ROS_WARN_THROTTLE(5.0, "[IROCBridge]: waiting for ROS data");
    }
    return;
  }


  if (got_uav_status){
    auto uav_status = sh_uav_status_.getMsg();
    parseLocalPosition(uav_status);
  }

  if (got_hw_api_gnss){
    auto hw_api_gnss = sh_hw_api_gnss_.getMsg();
    parseGlobalPosition(hw_api_gnss);
  }
  
  /* TODO: add more messages types */
  last_update_time_ = time_now;
}

//}

// | -------------------- support functions ------------------- |
//
/* parseLocalPosition() //{ */

void IROCBridge::parseLocalPosition(const mrs_msgs::UavStatusConstPtr &uav_status) {
  ROS_INFO_THROTTLE(1,"[IROCBridge]: LocalPosition: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", 
      uav_status->odom_x, uav_status->odom_y, uav_status->odom_z, uav_status->odom_hdg);

  const json json_msg = {
      {"x", uav_status->odom_x},
      {"y", uav_status->odom_y},
      {"z", uav_status->odom_z},
      {"heading", uav_status->odom_hdg},
  };
  sendJsonMessage("LocalPosition", json_msg);
}

//}

/* parseGlobalPosition() //{ */

void IROCBridge::parseGlobalPosition(const sensor_msgs::NavSatFixConstPtr &global_position) {
  ROS_INFO_THROTTLE(1,"[IROCBridge]: GlobalPosition: lat: %.2f, lon: %.2f, alt: %.2f", 
      global_position->latitude, global_position->longitude, global_position->altitude);

  const json json_msg = {
      {"latitude", global_position->latitude},
      {"longitude", global_position->longitude},
      {"altitude", global_position->altitude},
  };
  sendJsonMessage("GlobalPosition", json_msg);
}

//}

/* sendJsonMessage() //{ */

void IROCBridge::sendJsonMessage(const std::string& msg_type, const json& json_msg)
{
  const std::string url = "api/robot/telemetry/" + msg_type;
  const std::string body = json_msg.dump();
  const std::string content_type = "application/x-www-form-urlencoded";
  const auto res = http_client_->Patch(url, body, content_type);
  
  if (res)
    ROS_INFO_STREAM_THROTTLE(1.0, res->status << ": " << res->body);
  else
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to send PATCH request: " << to_string(res.error()));

  return;
}

//}

/* callService() //{ */

template <typename Svc_T>
IROCBridge::svc_call_res_t IROCBridge::callService(ros::ServiceClient& sc, typename Svc_T::Request req)
{
  typename Svc_T::Response res;
  if (sc.call(req, res))
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc_arm_.getService() << "\" with response \"" << res.message << "\".");
    return {true, res.message};
  }
  else
  {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM_THROTTLE(1.0, msg);
    return {false, msg};
  }
}

template <typename Svc_T>
IROCBridge::svc_call_res_t IROCBridge::callService(ros::ServiceClient& sc)
{
  return callService<Svc_T>(sc, {});
}

IROCBridge::svc_call_res_t IROCBridge::callService(ros::ServiceClient& sc, const bool val)
{
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}

//}

void IROCBridge::routine_death_check()
{
  ROS_INFO("[IROCBridge]: Death check routine running.");
  // to enable graceful exit, the server needs to be stopped
  const ros::WallDuration period(0.5);
  while (ros::ok())
    period.sleep();
  ROS_INFO("[IROCBridge]: Stopping the HTTP server.");
  http_srv_.stop();
  ROS_INFO("[IROCBridge]: Stopping the HTTP client.");
  http_client_->stop();
}

}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
