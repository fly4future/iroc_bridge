/* includes //{ */

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <nlohmann/json.hpp>
#include <httplib/httplib.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <mrs_msgs/UavStatus.h>
#include <mrs_msgs/UavDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/EstimationDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Path.h>

#include "iroc_bridge/json_var_parser.h"

//}

namespace iroc_bridge
{

using json = nlohmann::json;

using vec3_t = Eigen::Vector3d;
using vec4_t = Eigen::Vector4d;

/* class IROCBridge //{ */

class IROCBridge : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;

  std::thread th_http_srv_;
  httplib::Server http_srv_;
  std::unique_ptr<httplib::Client> http_client_;

  std::string _robot_name_;
  std::string _robot_type_;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::UavDiagnostics> sh_robot_diags_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;

  mrs_lib::SubscribeHandler<sensor_msgs::BatteryState> sh_battery_state_;

  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics> sh_estimation_diagnostics_;
  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_hw_api_gnss_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_control_manager_heading_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped> sh_hw_api_mag_heading_;

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;
  mrs_lib::SubscribeHandler<std_msgs::Float64> sh_control_manager_thrust;

  ros::ServiceClient sc_arm_;
  ros::ServiceClient sc_offboard_;
  ros::ServiceClient sc_land_;

  ros::Publisher pub_path_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | ------------------ Additional functions ------------------ |

  void parseRobotState(const mrs_msgs::UavDiagnostics::ConstPtr& uav_status);
  void parseRobotInfo(const sensor_msgs::BatteryState::ConstPtr& battery_state);
  void parseStateEstimationInfo(const mrs_msgs::EstimationDiagnostics::ConstPtr& estimation_diagnostics, const mrs_msgs::Float64Stamped::ConstPtr& local_heading, const sensor_msgs::NavSatFix::ConstPtr& global_position, const mrs_msgs::Float64Stamped::ConstPtr& global_heading);
  void parseControlInfo(const mrs_msgs::ControlManagerDiagnostics::ConstPtr& control_manager_diagnostics, const std_msgs::Float64::ConstPtr& thrust);
  void sendJsonMessage(const std::string& msg_type, const json& json_msg);

  void pathCallback(const httplib::Request&, httplib::Response& res);

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
  
  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROCBridge");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  param_loader.loadParam("robot_name", _robot_name_);
  param_loader.loadParam("robot_type", _robot_type_);
  const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  const auto url = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");

  if (!param_loader.loadedSuccessfully())
  {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  pub_path_ = nh_.advertise<mrs_msgs::Path>("path_out", 10);

  // | --------------------- service clients -------------------- |

  sc_arm_ = nh_.serviceClient<std_srvs::SetBool>("arm");
  sc_offboard_ = nh_.serviceClient<std_srvs::Trigger>("offboard");
  sc_land_ = nh_.serviceClient<std_srvs::Trigger>("land");

  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  //TODO: move this to separate methods to not clutter the initialization
  http_srv_.Get("/takeoff", [&](const httplib::Request&, httplib::Response& res)
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

  http_srv_.Get("/land", [&](const httplib::Request&, httplib::Response& res)
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "Calling land.");
        const auto resp = callService<std_srvs::Trigger>(sc_land_);
        if (!resp.call_success)
        {
          res.set_content(resp.message, "text/plain");
          return;
        }
        res.set_content("Landing.", "text/plain");
      });

  const httplib::Server::Handler hdlr_set_path = std::bind(&IROCBridge::pathCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_path", hdlr_set_path);

  th_http_srv_ = std::thread([&]()
      {
        http_srv_.listen(url, server_port);
      });
  th_http_srv_.detach();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "IROCBridge";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_robot_diags_ = mrs_lib::SubscribeHandler<mrs_msgs::UavDiagnostics>(shopts, "robot_diagnostics_in");
  sh_uav_status_ = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");

  // | ------------------------ RobotInfo ----------------------- |
  sh_battery_state_ = mrs_lib::SubscribeHandler<sensor_msgs::BatteryState>(shopts, "battery_state_in");

  // | ------------------- StateEstimationInfo ------------------ |
  sh_estimation_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "estimation_diagnostics_in");
  sh_hw_api_gnss_ = mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix>(shopts, "hw_api_gnss_in");
  sh_control_manager_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "control_manager_heading_in");
  sh_hw_api_mag_heading_ = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "hw_api_mag_heading_in");

  sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_control_manager_thrust = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "control_manager_thrust_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &IROCBridge::timerMain, this);
  th_death_check_ = std::thread(&IROCBridge::routine_death_check, this);
  th_death_check_.detach();

  // | --------------------- finish the init -------------------- |

  ROS_INFO("[IROCBridge]: initialized");
  ROS_INFO("[IROCBridge]: --------------------");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void IROCBridge::timerMain([[maybe_unused]] const ros::TimerEvent &event)
{

  if (sh_robot_diags_.newMsg())
    parseRobotState(sh_robot_diags_.getMsg());

  if (sh_battery_state_.newMsg())
    parseRobotInfo(sh_battery_state_.getMsg());

  if (sh_estimation_diagnostics_.newMsg() && sh_control_manager_heading_.newMsg() && sh_hw_api_gnss_.newMsg() && sh_hw_api_mag_heading_.newMsg())
    parseStateEstimationInfo(sh_estimation_diagnostics_.getMsg(), sh_control_manager_heading_.getMsg(), sh_hw_api_gnss_.getMsg(), sh_hw_api_mag_heading_.getMsg());

  if (sh_control_manager_diagnostics_.newMsg() && sh_control_manager_thrust.newMsg())
    parseControlInfo(sh_control_manager_diagnostics_.getMsg(), sh_control_manager_thrust.getMsg());
}

//}

// | -------------------- support functions ------------------- |

/* parseRobotState() method //{ */
void IROCBridge::parseRobotState(const mrs_msgs::UavDiagnostics::ConstPtr& robot_diags)
{
  ROS_INFO_STREAM_THROTTLE(1,"[IROCBridge]: Robot state: \"" << robot_diags->state << "\".");

  const json json_msg = {
      {"state", robot_diags->state},
  };
  sendJsonMessage("RobotState", json_msg);
}
//}

/* parseRobotInfo() //{ */

void IROCBridge::parseRobotInfo(const sensor_msgs::BatteryState::ConstPtr& battery_state)
{
  const json json_msg =
  {
    {"robot_name", _robot_name_},
    {"robot_type", _robot_type_},
    {"battery_state",
      {"voltage", battery_state->voltage},
      {"percentage", battery_state->percentage},
      {"wh_drained", "NOT_IMPLEMENTED"},
    },
    {"ready_to_start", "NOT_IMPLEMENTED"},
    {"problems", "NOT_IMPLEMENTED"},
  };
  sendJsonMessage("RobotInfo", json_msg);
}

//}

/* parseStateEstimationInfo() //{ */

void IROCBridge::parseStateEstimationInfo(const mrs_msgs::EstimationDiagnostics::ConstPtr& estimation_diagnostics, const mrs_msgs::Float64Stamped::ConstPtr& local_heading, const sensor_msgs::NavSatFix::ConstPtr& global_position, const mrs_msgs::Float64Stamped::ConstPtr& global_heading)
{
  const json json_msg =
  {
    {"estimation_frame", estimation_diagnostics->header.frame_id},
    {"local_pose",
      {"x", estimation_diagnostics->pose.position.x},
      {"y", estimation_diagnostics->pose.position.y},
      {"z", estimation_diagnostics->pose.position.z},
      {"heading", local_heading->value},
    },
    {"global_pose",
      {"latitude", global_position->latitude},
      {"longitude", global_position->longitude},
      {"altitude", global_position->altitude},
      {"heading", global_heading->value},
    },
    {"above_ground_level_height", estimation_diagnostics->agl_height},
    {"velocity",
      {"linear",
        {"x", estimation_diagnostics->velocity.linear.x},
        {"y", estimation_diagnostics->velocity.linear.y},
        {"z", estimation_diagnostics->velocity.linear.z},
      },
      {"angular",
        {"x", estimation_diagnostics->velocity.angular.x},
        {"y", estimation_diagnostics->velocity.angular.y},
        {"z", estimation_diagnostics->velocity.angular.z},
      },
    },
    {"acceleration",
      {"linear",
        {"x", estimation_diagnostics->acceleration.linear.x},
        {"y", estimation_diagnostics->acceleration.linear.y},
        {"z", estimation_diagnostics->acceleration.linear.z},
      },
      {"angular",
        {"x", estimation_diagnostics->acceleration.angular.x},
        {"y", estimation_diagnostics->acceleration.angular.y},
        {"z", estimation_diagnostics->acceleration.angular.z},
      },
    },
    {"running_estimators", estimation_diagnostics->running_state_estimators},
    {"switchable_estimators", estimation_diagnostics->switchable_state_estimators},
  };
  sendJsonMessage("StateEstimationInfo", json_msg);
}

//}

/* parseControlInfo() //{ */

void IROCBridge::parseControlInfo(const mrs_msgs::ControlManagerDiagnostics::ConstPtr& control_manager_diagnostics, const std_msgs::Float64::ConstPtr& thrust)
{
  const json json_msg =
  {
    {"active_controller", control_manager_diagnostics->active_controller},
    {"available_controllers", control_manager_diagnostics->available_controllers},
    {"active_tracker", control_manager_diagnostics->active_tracker},
    {"available_trackers", control_manager_diagnostics->available_trackers},
    {"thrust", thrust->data},
  };
  sendJsonMessage("ControlInfo", json_msg);
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

/* pathCallback() method //{ */
void IROCBridge::pathCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a path message JSON -> ROS.");
  json json_msg;
  try
  {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string frame_id;
  json points;
  const auto succ = parse_vars(json_msg, {{"frame_id", &frame_id}, {"points", &points}});
  if (!succ)
    return;

  if (!points.is_array())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
    return;
  }

  mrs_msgs::Path msg_path;
  msg_path.points.reserve(points.size());
  bool use_heading = false;
  for (const auto& el : points)
  {
    mrs_msgs::Reference ref;
    const auto succ = parse_vars(el, {{"x", &ref.position.x}, {"y", &ref.position.y}, {"z", &ref.position.z}});
    if (!succ)
      return;

    if (el.contains("heading"))
    {
      ref.heading = el.at("heading");
      use_heading = true;
    }
    msg_path.points.push_back(ref);
  }
  msg_path.header.stamp = ros::Time::now();
  msg_path.header.frame_id = frame_id;
  msg_path.fly_now = true;
  msg_path.use_heading = use_heading;
  pub_path_.publish(msg_path);
  ROS_INFO_STREAM("[IROCBridge]: Set a path with " << points.size() << " length.");
}
//}

/* routine_death_check() method //{ */
void IROCBridge::routine_death_check()
{
  // to enable graceful exit, the server needs to be stopped
  const ros::WallDuration period(0.5);
  while (ros::ok())
    period.sleep();
  ROS_INFO("[IROCBridge]: Stopping the HTTP server.");
  http_srv_.stop();
  ROS_INFO("[IROCBridge]: Stopping the HTTP client.");
  http_client_->stop();
}
//}

}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
