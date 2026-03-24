#pragma once

/* ROS */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/* mrs_lib */
#include <mrs_lib/node.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

/* ROS messages */
#include <mrs_msgs/msg/path.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/sensor_info.hpp>
#include <mrs_msgs/msg/safety_border.hpp>
#include <mrs_msgs/msg/point2_d.hpp>

/* ROS services */
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/set_safety_border_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/set_obstacle_srv.hpp>

/* Fleet manager interfaces */
#include <iroc_fleet_manager/srv/change_fleet_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/change_robot_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/get_world_origin_srv.hpp>
#include <iroc_fleet_manager/srv/get_safety_border_srv.hpp>
#include <iroc_fleet_manager/srv/get_obstacles_srv.hpp>
#include <iroc_fleet_manager/srv/get_mission_points_srv.hpp>
#include <iroc_fleet_manager/srv/upload_fleet_mission_srv.hpp>
#include <iroc_fleet_manager/action/execute_mission.hpp>

/* MRS diagnostics */
#include <mrs_robot_diagnostics/enums/robot_type.h>

/* IROC */
#include <iroc_common/result.h>

/* Eigen */
#include <Eigen/Core>

/* Third party */
#include <httplib/httplib.h>
#include "crow.h"
#include "crow/middlewares/cors.h"

/* STL */
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace iroc_bridge
{

using json = crow::json::wvalue;

using vec3_t = Eigen::Vector3d;
using vec4_t = Eigen::Vector4d;

using Mission           = iroc_fleet_manager::action::ExecuteMission;
using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;
using MissionClient     = rclcpp_action::Client<iroc_fleet_manager::action::ExecuteMission>;
using MissionGoalHandle = rclcpp_action::ClientGoalHandle<iroc_fleet_manager::action::ExecuteMission>;
typedef mrs_robot_diagnostics::robot_type_t robot_type_t;

class IROCBridge : public mrs_lib::Node {
public:
  IROCBridge(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void initialize(void);
  void shutdown();

  // | ---------------------- HTTP REST API --------------------- |
  std::thread                  th_http_srv_;
  crow::App<crow::CORSHandler> http_srv_;

  std::unique_ptr<httplib::Client> http_client_;

  using result_t = iroc_common::result_t;

  struct action_result_t
  {
    bool         success;
    std::string  message;
    crow::status status_code;
  };

  // | ---------------------- Command types --------------------- |
  enum class CommandType
  {
    Takeoff,
    Land,
    Hover,
    Home,
    Set_Origin,
    Set_SafetyBorder,
    Set_Obstacle,
    Unknown
  };

  enum class Change_SvC_T
  {
    FleetWaypoint,
    RobotWaypoint,
    FleetCoverage,
    RobotCoverage,
    RobotAutonomyTest
  };

  // | ---------------------- ROS parameters ------------------ |
  double max_linear_speed_;
  double max_heading_rate_;

  struct robot_handler_t
  {
    std::string                                                       robot_name;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>       sh_general_robot_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>    sh_state_estimation_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>            sh_control_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo> sh_collision_avoidance_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>                sh_uav_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>       sh_system_health_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SensorInfo>             sh_sensor_info;

    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_takeoff;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_hover;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_land;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                     sc_land_home;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>         sc_set_origin;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetSafetyBorderSrv>          sc_set_safety_area;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetObstacleSrv>              sc_set_obstacle;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> sc_velocity_reference;

    mrs_lib::PublisherHandler<mrs_msgs::msg::Path> pub_path;
  };

  struct robot_handlers_t
  {
    std::recursive_mutex         mtx;
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  std::map<std::string, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> robot_handler_t::*> trigger_command_handlers_ = {
      {"takeoff", &robot_handler_t::sc_takeoff},
      {"land", &robot_handler_t::sc_land},
      {"hover", &robot_handler_t::sc_hover},
      {"home", &robot_handler_t::sc_land_home}};

  // | ----------------------- main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();

  // | ----------------------- ROS Clients ----------------------- |
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv> sc_change_fleet_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> sc_change_robot_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>          sc_get_world_origin_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv>         sc_get_safety_border_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetObstaclesSrv>            sc_get_obstacles_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetMissionPointsSrv>        sc_get_mission_data_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv>      sc_upload_fleet_mission_;

  // | ----------------- action client callbacks ---------------- |

  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult &result);
  void missionFeedbackCallback(const Mission::Feedback::ConstSharedPtr feedback);

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr general_robot_info);
  void parseStateEstimationInfo(mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr state_estimation_info, const std::string &robot_name);
  void parseControlInfo(mrs_msgs::msg::ControlInfo::ConstSharedPtr control_info, const std::string &robot_name);
  void parseCollisionAvoidanceInfo(mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr collision_avoidance_info, const std::string &robot_name);
  void parseUavInfo(mrs_msgs::msg::UavInfo::ConstSharedPtr uav_info, const std::string &robot_name);
  void parseSensorInfo(mrs_msgs::msg::SensorInfo::ConstSharedPtr sensor_info, const std::string &robot_name);
  void parseSystemHealthInfo(mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr uav_info, const std::string &robot_name);

  void             sendJsonMessage(const std::string &msg_type, json &json_msg);
  void             sendFeedbackJsonMessage(json &json_msg);
  void             sendTelemetryJsonMessage(const std::string &type, json &json_msg);
  robot_handler_t *findRobotHandler(const std::string &robot_name, robot_handlers_t &robot_handlers);

  action_result_t commandAction(const std::vector<std::string> &robot_names, const std::string &command_type);

  template <typename ServiceType>
  action_result_t commandAction(const std::vector<std::string> &robot_names, mrs_lib::ServiceClientHandler<ServiceType> robot_handler_t::*handler_member,
                                const std::shared_ptr<typename ServiceType::Request> &request);

  // REST API callbacks
  crow::response setOriginCallback(const crow::request &req);
  crow::response getOriginCallback(const crow::request &req);
  crow::response setSafetyBorderCallback(const crow::request &req);
  crow::response getSafetyBorderCallback(const crow::request &req);
  crow::response setObstacleCallback(const crow::request &req);
  crow::response getObstaclesCallback(const crow::request &req);
  crow::response uploadMissionCallback(const crow::request &req);
  crow::response getMissionCallback(const crow::request &req);

  crow::response changeFleetMissionStateCallback(const crow::request &req, const std::string &type);
  crow::response changeRobotMissionStateCallback(const crow::request &req, const std::string &robot_name, const std::string &type);

  crow::response availableRobotsCallback(const crow::request &req);
  crow::response commandCallback(const crow::request &req, const std::string &command_type, std::optional<std::string> robot_name);

  // Websocket callbacks
  void remoteControlCallback(crow::websocket::connection &conn, const std::string &data, bool is_binary);

  // some helper method overloads
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                       const std::shared_ptr<typename ServiceType::Response> &response);

  std::thread                  th_death_check_;
  std::thread                  th_telemetry_check_;
  void                         routine_death_check();
  crow::websocket::connection *active_telemetry_connection_ = nullptr;
  std::mutex                   mtx_telemetry_connections_;
  crow::websocket::connection *active_feedback_connection_ = nullptr;
  std::mutex                   mtx_feedback_connections_;

  std::shared_ptr<MissionClient> mission_client_;
  MissionGoalHandle::SharedPtr   current_goal_handle_;
  std::mutex                     mtx_current_goal_handle_;

  // Latlon origin
  mrs_msgs::msg::Point2D world_origin_;
};

} // namespace iroc_bridge
