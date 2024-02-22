/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

/* custom msgs of MRS group */
#include <mrs_msgs/UavStatus.h>

//}

namespace ros2json_bridge
{

/* class Ros2JsonBridge //{ */

class Ros2JsonBridge : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  // | ---------------------- ROS subscribers --------------------- |
  mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>     sh_uav_status_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

};
//}

/* onInit() //{ */

void Ros2JsonBridge::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "Ros2JsonBridge");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }
  
  param_loader.addYamlFileFromParam("config");

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Ros2JsonBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Ros2JsonBridge";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_uav_status_      = mrs_lib::SubscribeHandler<mrs_msgs::UavStatus>(shopts, "uav_status_in");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &Ros2JsonBridge::timerMain, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[Ros2JsonBridge]: initialized");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void Ros2JsonBridge::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  bool got_uav_status     = sh_uav_status_.hasMsg();

  if (!got_uav_status) {
    ROS_WARN_THROTTLE(5.0, "[Ros2JsonBridge]: waiting for data: UavStatus=%s", got_uav_status ? "true" : "FALSE");
    return;
  }

  auto uav_status = sh_uav_status_.getMsg();

}

//}

//}

}  // namespace ros2json_bridge

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros2json_bridge::Ros2JsonBridge, nodelet::Nodelet);
