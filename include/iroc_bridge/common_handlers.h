#ifndef IROC_BRIDGE_COMMON_HANDLERS_H
#define IROC_BRIDGE_COMMON_HANDLERS_H

#include <mrs_lib/mutex.h>
#include <memory.h>
#include <string>
#include <unordered_map>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/enums/robot_type.h>

namespace iroc_bridge {

// Note: ConstPtr type is a typedef for a shared pointer, 
// specifically boost::shared_ptr in ROS 1

struct CommonRobotHandler_t {
  std::string robot_name;
  mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr
      general_robot_info;
  mrs_robot_diagnostics::StateEstimationInfo::ConstPtr
      state_estimation_info;
  mrs_robot_diagnostics::ControlInfo::ConstPtr control_info;
  mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr
      collision_avoidance_info;
  mrs_robot_diagnostics::UavInfo::ConstPtr uav_info;
  mrs_robot_diagnostics::SystemHealthInfo::ConstPtr
      system_health_info;
};

struct CommonRobotHandlers_t {
  std::unordered_map<std::string,CommonRobotHandler_t> handlers_map;

};

struct CommonHandlers_t {
  CommonRobotHandlers_t robot_handlers;
  ros::NodeHandle parent_nh;
};

} // namespace iroc_bridge

#endif // IROC_BRIDGE_COMMON_HANDLERS_H
