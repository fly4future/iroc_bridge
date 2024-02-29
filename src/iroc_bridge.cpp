/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <nlohmann/json.hpp>

#include <sensor_msgs/NavSatFix.h>

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

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus> sh_uav_status_;
  mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sh_hw_api_gnss_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);
  double     _main_timer_rate_;

  // | ------------------ Additional functions ------------------ |

  void parseLocalPosition(const mrs_msgs::UavStatusConstPtr &uav_status);
  void parseGlobalPosition(const sensor_msgs::NavSatFixConstPtr &global_position);
  void sendJsonMessage(const std::string &msg_type, const json &json_msg);
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

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

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

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &IROCBridge::timerMain, this);

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

void IROCBridge::sendJsonMessage(const std::string &msg_type, const json &json_msg) {
  // Fill calls for restAPI
  // TODO: Add communication with restAPI
  /* const int print_indent = 2; */
  /* ROS_INFO("[IROCBridge]: sending \"%s\" msg: \n%s", msg_type.c_str(), json_msg.dump(print_indent).c_str()); */
  return;
}

//}

//}


}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
