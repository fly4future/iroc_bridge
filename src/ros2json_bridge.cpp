/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ReferenceStamped.h>

//}

namespace ros2json_bridge
{

/* class Ros2JsonBridge //{ */

class Ros2JsonBridge : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  // | -------------------------- flags ------------------------- |

  /* is set to true when the nodelet is initialized, useful for rejecting callbacks that are called before the node is initialized */
  std::atomic<bool> is_initialized_ = false;

  /* by default, the nodelet is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> have_odom_ = false;

  /* ROS messages which store the current reference and odometry */
  mrs_msgs::ReferenceStamped ref_;
  nav_msgs::Odometry         current_odom_;

  // | ---------------------- ROS subscribers --------------------- |
  ros::Subscriber sub_odom_;
  void            callbackOdom(const nav_msgs::Odometry& msg);

};
//}

/* onInit() //{ */

void Ros2JsonBridge::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | -------- initialize a subscriber for UAV odometry -------- |
  sub_odom_ = nh.subscribe("odom_in", 10, &Ros2JsonBridge::callbackOdom, this, ros::TransportHints().tcpNoDelay());

  is_initialized_   = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */


void Ros2JsonBridge::callbackOdom(const nav_msgs::Odometry& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }
  // | -------------- save the current UAV odometry ------------- |
  current_odom_ = msg;
  have_odom_    = true;
}

//}

//}

}  // namespace ros2json_bridge

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros2json_bridge::Ros2JsonBridge, nodelet::Nodelet);
