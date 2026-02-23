#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/node.h>
// ROS message includes
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
/* custom msgs of MRS group */
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

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iroc_fleet_manager/srv/change_fleet_mission_state_srv.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/set_safety_border_srv.hpp>
#include <mrs_msgs/srv/velocity_reference_stamped_srv.hpp>
#include <mrs_msgs/srv/set_obstacle_srv.hpp>

#include <iroc_fleet_manager/action/execute_mission.hpp>
#include <iroc_fleet_manager/srv/change_robot_mission_state_srv.hpp>
#include <iroc_fleet_manager/srv/get_world_origin_srv.hpp>
#include <iroc_fleet_manager/srv/get_safety_border_srv.hpp>
#include <iroc_fleet_manager/srv/get_obstacles_srv.hpp>
#include <iroc_fleet_manager/srv/get_mission_points_srv.hpp>
#include <iroc_fleet_manager/srv/upload_fleet_mission_srv.hpp>

#include <mrs_robot_diagnostics/enums/robot_type.h>

// General includes
#include <unistd.h>
#include <iostream>
#include <future>
#include <unordered_map>
#include <httplib/httplib.h>
#include <string>
#include "crow.h"
#include "crow/middlewares/cors.h"

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

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
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void initialize(void);
  void shutdown();

  // | ---------------------- HTTP REST API --------------------- |
  std::thread th_http_srv_;
  crow::App<crow::CORSHandler> http_srv_;

  std::unique_ptr<httplib::Client> http_client_;

  struct result_t
  {
    bool success;
    std::string message;
  };

  struct action_result_t
  {
    bool success;
    std::string message;
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
    std::string robot_name;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo> sh_general_robot_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo> sh_state_estimation_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo> sh_control_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo> sh_collision_avoidance_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo> sh_uav_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo> sh_system_health_info;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::SensorInfo> sh_sensor_info;

    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_takeoff;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_hover;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_land;
    mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sc_land_home;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sc_set_origin;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetSafetyBorderSrv> sc_set_safety_area;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetObstacleSrv> sc_set_obstacle;
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv> sc_velocity_reference;

    mrs_lib::PublisherHandler<mrs_msgs::msg::Path> pub_path;
  };

  struct robot_handlers_t
  {
    std::recursive_mutex mtx;
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  std::map<std::string, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> robot_handler_t::*> trigger_command_handlers_ = {
      {"takeoff", &robot_handler_t::sc_takeoff},
      {"land", &robot_handler_t::sc_land},
      {"hover", &robot_handler_t::sc_hover},
      {"home", &robot_handler_t::sc_land_home}};

  // | ----------------------- main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  void timerMain();

  // | ----------------------- ROS Clients ----------------------- |
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv> sc_change_fleet_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> sc_change_robot_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetWorldOriginSrv> sc_get_world_origin_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv> sc_get_safety_border_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetObstaclesSrv> sc_get_obstacles_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetMissionPointsSrv> sc_get_mission_data_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv> sc_upload_fleet_mission_;

  // | ----------------- action client callbacks ---------------- |

  // // Mission callbacks
  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult &result);
  void missionFeedbackCallback(const Mission::Feedback::ConstSharedPtr feedback);

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr general_robot_info, const std::string &robot_name);
  void parseStateEstimationInfo(mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr state_estimation_info, const std::string &robot_name);
  void parseControlInfo(mrs_msgs::msg::ControlInfo::ConstSharedPtr control_info, const std::string &robot_name);
  void parseCollisionAvoidanceInfo(mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr collision_avoidance_info, const std::string &robot_name);
  void parseUavInfo(mrs_msgs::msg::UavInfo::ConstSharedPtr uav_info, const std::string &robot_name);
  void parseSensorInfo(mrs_msgs::msg::SensorInfo::ConstSharedPtr sensor_info, const std::string &robot_name);
  void parseSystemHealthInfo(mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr uav_info, const std::string &robot_name);

  void sendJsonMessage(const std::string &msg_type, json &json_msg);
  void sendTelemetryJsonMessage(const std::string &type, json &json_msg);
  robot_handler_t *findRobotHandler(const std::string &robot_name, robot_handlers_t &robot_handlers);

  action_result_t commandAction(const std::vector<std::string> &robot_names, const std::string &command_type);

  template <typename ServiceType>
  action_result_t commandAction(const std::vector<std::string> &robot_names, mrs_lib::ServiceClientHandler<ServiceType> robot_handler_t::*handler_member,
                                const std::shared_ptr<typename ServiceType::Request> &request);

  // REST API callbacks
  crow::response pathCallback(const crow::request &req);
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

  std::thread th_death_check_;
  std::thread th_telemetry_check_;
  void routine_death_check();
  crow::websocket::connection *active_telemetry_connection_ = nullptr;
  std::mutex mtx_telemetry_connections_;

  std::shared_ptr<MissionClient> mission_client_;
  MissionGoalHandle::SharedPtr current_goal_handle_;
  std::mutex mtx_current_goal_handle_;

  // Latlon origin
  mrs_msgs::msg::Point2D world_origin_;
};

IROCBridge::IROCBridge(rclcpp::NodeOptions options) : mrs_lib::Node("IROCBridge", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();
}

void IROCBridge::initialize() {

  std::vector<char> hostname(1024);
  std::string hostname_result;
  gethostname(hostname.data(), hostname.size()) == 0 ? hostname_result = std::string(hostname.data()) : hostname_result = "unknown";
  RCLCPP_INFO(node_->get_logger(), "Hostname: %s", hostname_result.c_str());

  mrs_lib::ParamLoader param_loader(node_, "IROCBridge");
  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  // Custom config loaded first to have the priority, if not given it takes the default config file
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("network_config");
  param_loader.addYamlFileFromParam("config");

  const auto robot_names = param_loader.loadParam2<std::vector<std::string>>("network/robot_names");

  // Remove ground-station hostname from robot names
  std::string hostname_str(hostname.data());
  std::vector<std::string> filtered_robot_names = robot_names;

  auto it = std::remove(filtered_robot_names.begin(), filtered_robot_names.end(), hostname_str);
  filtered_robot_names.erase(it, filtered_robot_names.end());

  // Arguments set config file
  const auto client_url  = param_loader.loadParam2<std::string>("iroc_bridge/client_url");
  const auto client_port = param_loader.loadParam2<int>("iroc_bridge/client_port");
  const auto server_port = param_loader.loadParam2<int>("iroc_bridge/server_port");

  const auto main_timer_rate       = param_loader.loadParam2<double>("iroc_bridge/main_timer_rate");
  const auto _http_server_threads_ = param_loader.loadParam2<double>("iroc_bridge/http_server_threads");
  const auto no_message_timeout    = param_loader.loadParam2<rclcpp::Duration>("iroc_bridge/no_message_timeout");

  max_linear_speed_ = param_loader.loadParam2<double>("iroc_bridge/remote_control_limits/max_linear_speed");
  max_heading_rate_ = param_loader.loadParam2<double>("iroc_bridge/remote_control_limits/max_heading_rate");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------- HTTP REST API callbacks ---------------- |
  // HTTP Client
  http_client_ = std::make_unique<httplib::Client>(client_url, client_port);

  // HTTP Server
  // Do we need this (set_path)?
  CROW_ROUTE(http_srv_, "/set_path").methods(crow::HTTPMethod::Post)([this](const crow::request &req) { return pathCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/world-origin").methods(crow::HTTPMethod::Post)([this](const crow::request &req) { return setOriginCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/borders").methods(crow::HTTPMethod::Post)([this](const crow::request &req) { return setSafetyBorderCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/obstacles").methods(crow::HTTPMethod::Post)([this](const crow::request &req) { return setObstacleCallback(req); });
  CROW_ROUTE(http_srv_, "/mission").methods(crow::HTTPMethod::Post)([this](const crow::request &req) { return uploadMissionCallback(req); });

  // Getters
  CROW_ROUTE(http_srv_, "/safety-area/world-origin").methods(crow::HTTPMethod::Get)([this](const crow::request &req) { return getOriginCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/borders").methods(crow::HTTPMethod::Get)([this](const crow::request &req) { return getSafetyBorderCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/obstacles").methods(crow::HTTPMethod::Get)([this](const crow::request &req) { return getObstaclesCallback(req); });
  CROW_ROUTE(http_srv_, "/mission").methods(crow::HTTPMethod::Get)([this](const crow::request &req) { return getMissionCallback(req); });

  // Missions
  // TODO: CROW_REGEX_ROUTE(http_srv_, R"(/fleet/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/mission/<string>").methods(crow::HTTPMethod::Post)([this](const crow::request &req, const std::string &type) {
    return changeFleetMissionStateCallback(req, type);
  });
  // TODO: CROW_REGEX_ROUTE(http_srv_, R"(/robots/(\w+)/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/robots/<string>/mission/<string>")
      .methods(crow::HTTPMethod::Post)([this](const crow::request &req, const std::string &robot_name, const std::string &type) {
        return changeRobotMissionStateCallback(req, robot_name, type);
      });

  // Available robots endpoint
  CROW_ROUTE(http_srv_, "/robots").methods(crow::HTTPMethod::Get)([this](const crow::request &req) { return availableRobotsCallback(req); });

  // Command endpoints with robot name in the path (land, takeoff, hover, home)
  CROW_ROUTE(http_srv_, "/robots/<string>/<string>")
      .methods(crow::HTTPMethod::Post)([this](const crow::request &req, const std::string &robot_name, const std::string &command_type) {
        return commandCallback(req, command_type, robot_name);
      });

  // Command endpoint for all robots (land, takeoff, hover, home)
  CROW_ROUTE(http_srv_, "/robots/<string>").methods(crow::HTTPMethod::Post)([this](const crow::request &req, const std::string &command_type) {
    return commandCallback(req, command_type, std::nullopt);
  });

  // Remote control websocket
  CROW_WEBSOCKET_ROUTE(http_srv_, "/rc")
      .onopen([&](crow::websocket::connection &conn) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "New remote control websocket connection: " << &conn);
        RCLCPP_INFO_STREAM(node_->get_logger(), "New remote control websocket connection: " << conn.get_remote_ip());
      })
      .onclose([&](crow::websocket::connection &conn, const std::string &reason, int code) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Websocket connection " << conn.get_remote_ip() << " closed: " << reason);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Websocket connection " << &conn << " closed: " << reason << "code: " << code);
      })
      .onmessage([this](crow::websocket::connection &conn, const std::string &data, bool is_binary) { return remoteControlCallback(conn, data, is_binary); });

  // Telemetry websocket
  CROW_WEBSOCKET_ROUTE(http_srv_, "/telemetry")
      .onopen([&](crow::websocket::connection &conn) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "New telemetry websocket connection: " << conn.get_remote_ip());
        RCLCPP_INFO_STREAM(node_->get_logger(), "New telemetry websocket connection: " << &conn);
        std::scoped_lock lock(mtx_telemetry_connections_);
        active_telemetry_connection_ = &conn;
      })
      .onclose([&](crow::websocket::connection &conn, const std::string &reason, int code) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Websocket connection " << conn.get_remote_ip() << " closed: " << reason);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Websocket connection " << &conn << " closed: " << reason << "code: " << code);
        std::scoped_lock lock(mtx_telemetry_connections_);
        if (active_telemetry_connection_ == &conn) {
          active_telemetry_connection_ = nullptr;
        }
      });

  th_http_srv_ = std::thread([&]() { http_srv_.loglevel(crow::LogLevel::ERROR).port(server_port).concurrency(_http_server_threads_).run(); });
  RCLCPP_INFO_STREAM(node_->get_logger(), "HTTP server started on port " << server_port);
  th_http_srv_.detach();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  shopts.node_name          = "IROCBridge";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  // populate the robot handlers vector
  {
    std::scoped_lock lck(robot_handlers_.mtx);

    robot_handlers_.handlers.reserve(filtered_robot_names.size());
    for (const auto &robot_name : filtered_robot_names) {
      robot_handler_t robot_handler;
      // To share with fleet manager and planners
      robot_handler.robot_name = robot_name;
      // Saving only the robot names to fill up the rest during the parsing of the messages
      const std::string general_robot_info_topic_name = "/" + robot_name + "/general_robot_info_in";
      robot_handler.sh_general_robot_info             = mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, general_robot_info_topic_name);

      const std::string state_estimation_info_topic_name = "/" + robot_name + "/state_estimation_info_in";
      robot_handler.sh_state_estimation_info = mrs_lib::SubscriberHandler<mrs_msgs::msg::StateEstimationInfo>(shopts, state_estimation_info_topic_name);

      const std::string control_info_topic_name = "/" + robot_name + "/control_info_in";
      robot_handler.sh_control_info             = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>(shopts, control_info_topic_name);

      const std::string collision_avoidance_info_topic_name = "/" + robot_name + "/collision_avoidance_info_in";
      robot_handler.sh_collision_avoidance_info =
          mrs_lib::SubscriberHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(shopts, collision_avoidance_info_topic_name);

      const std::string uav_info_topic_name = "/" + robot_name + "/uav_info_in";
      robot_handler.sh_uav_info             = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavInfo>(shopts, uav_info_topic_name);

      const std::string system_health_info_topic_name = "/" + robot_name + "/system_health_info_in";
      robot_handler.sh_system_health_info             = mrs_lib::SubscriberHandler<mrs_msgs::msg::SystemHealthInfo>(shopts, system_health_info_topic_name);

      // No timeout for sensor info
      shopts.no_message_timeout = mrs_lib::no_timeout;

      const std::string sensor_info_topic_name = "/" + robot_name + "/sensor_info_in";
      robot_handler.sh_sensor_info             = mrs_lib::SubscriberHandler<mrs_msgs::msg::SensorInfo>(shopts, sensor_info_topic_name);

      // sc_set_world_origin_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/set_world_origin_out", cbkgrp_sc_);
      // robot_handler.sc_takeoff = node_.serviceClient<std_srvs::Trigger>("/" + robot_name + node_.resolveName("svc/takeoff"));
      const std::string takeoff_service_name = "/" + robot_name + "/takeoff_svc_in";
      robot_handler.sc_takeoff               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, takeoff_service_name, cbkgrp_sc_);

      const std::string hover_service_name = "/" + robot_name + "/hover_svc_in";
      robot_handler.sc_hover               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, hover_service_name, cbkgrp_sc_);


      const std::string land_service_name = "/" + robot_name + "/land_svc_in";
      robot_handler.sc_land               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, land_service_name, cbkgrp_sc_);

      const std::string land_home_service_name = "/" + robot_name + "/land_home_svc_in";
      robot_handler.sc_land_home               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, land_home_service_name, cbkgrp_sc_);

      const std::string set_origin_service_name = "/" + robot_name + "/set_origin_svc_in";
      robot_handler.sc_set_origin               = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, set_origin_service_name, cbkgrp_sc_);

      const std::string set_safety_area_service_name = "/" + robot_name + "/set_safety_area_svc_in";
      robot_handler.sc_set_safety_area = mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetSafetyBorderSrv>(node_, set_safety_area_service_name, cbkgrp_sc_);

      const std::string set_obstacle_service_name = "/" + robot_name + "/set_obstacle_svc_in";
      robot_handler.sc_set_obstacle               = mrs_lib::ServiceClientHandler<mrs_msgs::srv::SetObstacleSrv>(node_, set_obstacle_service_name, cbkgrp_sc_);

      const std::string velocity_reference_service_name = "/" + robot_name + "/velocity_reference_svc_in";
      robot_handler.sc_velocity_reference =
          mrs_lib::ServiceClientHandler<mrs_msgs::srv::VelocityReferenceStampedSrv>(node_, velocity_reference_service_name, cbkgrp_sc_);

      // | ----------------------- publishers ----------------------- |
      const std::string path_topic_name = "/" + robot_name + "/path_out";
      robot_handler.pub_path            = mrs_lib::PublisherHandler<mrs_msgs::msg::Path>(node_, path_topic_name);

      // move is necessary because copy construction of the subscribe handlers is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }

  shopts.no_message_timeout      = mrs_lib::no_timeout;
  sc_change_fleet_mission_state_ =
      mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv>(node_, "~/change_fleet_mission_state_svc_in", cbkgrp_sc_);
  sc_change_robot_mission_state_ =
      mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv>(node_, "~/change_robot_mission_state_svc_in", cbkgrp_sc_);
  sc_get_world_origin_     = mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>(node_, "~/get_world_origin_svc_in", cbkgrp_sc_);
  sc_get_safety_border_    = mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv>(node_, "~/get_safety_border_svc_in", cbkgrp_sc_);
  sc_get_obstacles_        = mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetObstaclesSrv>(node_, "~/get_obstacles_svc_in", cbkgrp_sc_);
  sc_get_mission_data_     = mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetMissionPointsSrv>(node_, "~/get_mission_data_svc_in", cbkgrp_sc_);
  sc_upload_fleet_mission_ = mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv>(node_, "~/upload_fleet_mission_svc_in", cbkgrp_sc_);

  /* // | --------------------- action clients --------------------- | */

  // Mission action client
  const std::string action_client_topic = "~/mission_action_client_in";
  mission_client_                       = rclcpp_action::create_client<iroc_fleet_manager::action::ExecuteMission>(node_, action_client_topic);
  RCLCPP_INFO(node_->get_logger(), "Created action client on topic \'ac/mission\' -> \'%s\'", action_client_topic.c_str());

  // | ------------------------- timers ------------------------- |
  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), [this]() { this->timerMain(); });
  }

  th_death_check_ = std::thread(&IROCBridge::routine_death_check, this);
  th_death_check_.detach();

  // | --------------------- finish the init -------------------- |
  RCLCPP_INFO(node_->get_logger(), "initialized");

  RCLCPP_INFO(node_->get_logger(), R"(
   ___ ____   ___   ____ ____       _     _
  |_ _|  _ \ / _ \ / ___| __ ) _ __(_) __| | __ _  ___
   | || |_) | | | | |   |  _ \| '__| |/ _` |/ _` |/ _ \ 
   | ||  _ <| |_| | |___| |_) | |  | | (_| | (_| |  __/
  |___|_| \_\\___/ \____|____/|_|  |_|\__,_|\__, |\___|
  )");
}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

void IROCBridge::timerMain() {
  std::scoped_lock lck(robot_handlers_.mtx);

  // Parsing the messages to send in telemetry
  for (auto &rh : robot_handlers_.handlers) {
    const auto &robot_name = rh.robot_name;

    if (rh.sh_general_robot_info.newMsg()) {
      parseGeneralRobotInfo(rh.sh_general_robot_info.getMsg(), robot_name);
    }

    if (rh.sh_state_estimation_info.newMsg()) {
      parseStateEstimationInfo(rh.sh_state_estimation_info.getMsg(), robot_name);
    }

    if (rh.sh_control_info.newMsg()) {
      parseControlInfo(rh.sh_control_info.getMsg(), robot_name);
    }

    if (rh.sh_collision_avoidance_info.newMsg()) {
      parseCollisionAvoidanceInfo(rh.sh_collision_avoidance_info.getMsg(), robot_name);
    }

    if (rh.sh_uav_info.newMsg()) {
      parseUavInfo(rh.sh_uav_info.getMsg(), robot_name);
    }

    if (rh.sh_system_health_info.newMsg()) {
      parseSystemHealthInfo(rh.sh_system_health_info.getMsg(), robot_name);
    }

    if (rh.sh_sensor_info.newMsg()) {
      parseSensorInfo(rh.sh_sensor_info.getMsg(), robot_name);
    }
  }
}

// --------------------------------------------------------------
// |                  action client callbacks                   |
// --------------------------------------------------------------

void IROCBridge::missionDoneCallback(const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult &wrapped_result) {

  std::scoped_lock lck(mtx_current_goal_handle_);
  current_goal_handle_.reset();

  auto result = wrapped_result.result;

  // TODO fill properly
  switch (wrapped_result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO_STREAM(node_->get_logger(), "Fleet manager mission action server finished successfully");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_WARN_STREAM(node_->get_logger(), "Fleet manager mission action server was aborted");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_WARN_STREAM(node_->get_logger(), "Fleet manager mission action server was canceled");
    break;
  default:
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown result code from Mission Action server");
    break;
  }

  json robot_results = json::list();

  for (size_t i = 0; i < result->robot_results.size(); i++) {
    robot_results[i] = {{"robot_name", result->robot_results[i].name},
                        {"success", static_cast<bool>(result->robot_results[i].success)},
                        {"message", result->robot_results[i].message}};
  }

  // Create the main JSON object
  json json_msg = {{"success", static_cast<bool>(result->success)}, {"message", result->message}, {"robot_results", robot_results}};

  sendJsonMessage("results", json_msg);
}

void IROCBridge::missionFeedbackCallback(const Mission::Feedback::ConstSharedPtr feedback) {

  auto robot_feedbacks = feedback->info.robot_feedbacks;

  // Create a list for robot feedback
  json json_msgs = json::list();

  // Collect each robot feedback and create a json for each
  for (size_t i = 0; i < robot_feedbacks.size(); i++) {
    const auto &rfb = robot_feedbacks[i];

    json robot_json = {{"robot_name", rfb.name},
                       {"message", rfb.message},
                       {"mission_progress", rfb.mission_progress},
                       {"current_goal", rfb.goal_idx},
                       {"distance_to_goal", rfb.distance_to_closest_goal},
                       {"goal_estimated_arrival_time", rfb.goal_estimated_arrival_time},
                       {"goal_progress", rfb.goal_progress},
                       {"distance_to_finish", rfb.distance_to_finish},
                       {"finish_estimated_arrival_time", rfb.finish_estimated_arrival_time}};

    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "Mission feedback for robot: " << rfb.name << ", message: " << rfb.message << ", progress: " << rfb.mission_progress
                                                       << ", current goal: " << rfb.goal_idx << ", distance to goal: " << rfb.distance_to_closest_goal
                                                       << ", goal estimated arrival time: " << rfb.goal_estimated_arrival_time
                                                       << ", goal progress: " << rfb.goal_progress << ", distance to finish: " << rfb.distance_to_finish
                                                       << ", finish estimated arrival time: " << rfb.finish_estimated_arrival_time);

    // Add to the list at index i
    json_msgs[i] = std::move(robot_json);
  }

  // Create the main JSON message
  json json_msg = {{"progress", feedback->info.progress}, {"mission_state", feedback->info.state}, {"message", feedback->info.message}, {"robots", json_msgs}};

  sendTelemetryJsonMessage("MissionFeedback", json_msg);
}

// --------------------------------------------------------------
// |                 parsing and output methods                 |
// --------------------------------------------------------------

void IROCBridge::parseGeneralRobotInfo(mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr general_robot_info, const std::string &robot_name) {

  json json_msg = {
      {"robot_name", general_robot_info->robot_name},
      {"robot_type", general_robot_info->robot_type},
      {"battery_state",
       {{"voltage", general_robot_info->battery_state.voltage},
        {"percentage", general_robot_info->battery_state.percentage},
        {"wh_drained", general_robot_info->battery_state.wh_drained}}},
      {"ready_to_start", general_robot_info->ready_to_start},
      {"problems_preventing_start", json::list(general_robot_info->problems_preventing_start.begin(), general_robot_info->problems_preventing_start.end())},
      {"errors", json::list(general_robot_info->errors.begin(), general_robot_info->errors.end())}};

  sendTelemetryJsonMessage("GeneralRobotInfo", json_msg);
}

void IROCBridge::parseStateEstimationInfo(mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr state_estimation_info, const std::string &robot_name) {
  // Create the JSON structure using initializer lists
  json json_msg = {
      {"robot_name", robot_name},
      {"estimation_frame", state_estimation_info->header.frame_id},
      {"above_ground_level_height", state_estimation_info->above_ground_level_height},
      {"current_estimator", state_estimation_info->current_estimator},

      {"local_pose",
       {{"x", state_estimation_info->local_pose.position.x},
        {"y", state_estimation_info->local_pose.position.y},
        {"z", state_estimation_info->local_pose.position.z},
        {"heading", state_estimation_info->local_pose.heading}}},

      {"global_pose",
       {{"latitude", state_estimation_info->global_pose.position.x},
        {"longitude", state_estimation_info->global_pose.position.y},
        {"altitude", state_estimation_info->global_pose.position.z},
        {"heading", state_estimation_info->global_pose.heading}}},

      {"velocity",
       {{"linear",
         {{"x", state_estimation_info->velocity.linear.x}, {"y", state_estimation_info->velocity.linear.y}, {"z", state_estimation_info->velocity.linear.z}}},
        {"angular",
         {{"x", state_estimation_info->velocity.angular.x},
          {"y", state_estimation_info->velocity.angular.y},
          {"z", state_estimation_info->velocity.angular.z}}}}},

      {"acceleration",
       {{"linear",
         {{"x", state_estimation_info->acceleration.linear.x},
          {"y", state_estimation_info->acceleration.linear.y},
          {"z", state_estimation_info->acceleration.linear.z}}},
        {"angular",
         {{"x", state_estimation_info->acceleration.angular.x},
          {"y", state_estimation_info->acceleration.angular.y},
          {"z", state_estimation_info->acceleration.angular.z}}}}},

      {"running_estimators", {json::list(state_estimation_info->running_estimators.begin(), state_estimation_info->running_estimators.end())}},

      {"switchable_estimators", {json::list(state_estimation_info->switchable_estimators.begin(), state_estimation_info->switchable_estimators.end())}}};

  sendTelemetryJsonMessage("StateEstimationInfo", json_msg);
}

void IROCBridge::parseControlInfo(mrs_msgs::msg::ControlInfo::ConstSharedPtr control_info, const std::string &robot_name) {
  json json_msg = {{"robot_name", robot_name},
                   {"active_controller", control_info->active_controller},
                   {"available_controllers", json::list(control_info->available_controllers.begin(), control_info->available_controllers.end())},
                   {"active_tracker", control_info->active_tracker},
                   {"available_trackers", json::list(control_info->available_trackers.begin(), control_info->available_trackers.end())},
                   {"thrust", control_info->thrust}};

  sendTelemetryJsonMessage("ControlInfo", json_msg);
}

void IROCBridge::parseCollisionAvoidanceInfo(mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr collision_avoidance_info, const std::string &robot_name) {
  json json_msg = {
      {"robot_name", robot_name},
      {"collision_avoidance_enabled", collision_avoidance_info->collision_avoidance_enabled},
      {"avoiding_collision", collision_avoidance_info->avoiding_collision},
      {"other_robots_visible", json::list(collision_avoidance_info->other_robots_visible.begin(), collision_avoidance_info->other_robots_visible.end())},
  };

  sendTelemetryJsonMessage("CollisionAvoidanceInfo", json_msg);
}

void IROCBridge::parseUavInfo(mrs_msgs::msg::UavInfo::ConstSharedPtr uav_info, const std::string &robot_name) {
  json json_msg = {{"robot_name", robot_name},
                   {"armed", uav_info->armed},
                   {"offboard", uav_info->offboard},
                   {"flight_state", uav_info->flight_state},
                   {"flight_duration", uav_info->flight_duration},
                   {"mass_nominal", uav_info->mass_nominal}};

  sendTelemetryJsonMessage("UavInfo", json_msg);
}

void IROCBridge::parseSensorInfo(mrs_msgs::msg::SensorInfo::ConstSharedPtr sensor_info, const std::string &robot_name) {

  crow::json::rvalue json_msg = crow::json::load(sensor_info->details);
  if (!json_msg) {
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Could not parse sensor details JSON string: " << sensor_info->details);

    return;
  }
  json telemetry_msg = {{"robot_name", robot_name}, {"sensor_type", sensor_info->type}, {"details", json_msg}};

  sendTelemetryJsonMessage("SensorInfo", telemetry_msg);
}

void IROCBridge::parseSystemHealthInfo(mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr system_health_info, const std::string &robot_name) {
  // Create arrays for node_cpu_loads
  json node_cpu_loads = json::list();
  for (size_t i = 0; i < system_health_info->node_cpu_loads.size(); i++) {
    const auto &node_cpu_load = system_health_info->node_cpu_loads[i];

    // Create a nested array for each node_cpu_load using initializer list
    json node_entry = json::list({node_cpu_load.node_name, node_cpu_load.cpu_load});

    node_cpu_loads[i] = std::move(node_entry);
  }

  // Create array for available_sensors
  json available_sensors = json::list();
  for (size_t i = 0; i < system_health_info->available_sensors.size(); i++) {
    const auto &available_sensor = system_health_info->available_sensors[i];

    // Create an object for each required_sensor using initializer list
    available_sensors[i] = {
        {"name", available_sensor.name}, {"status", available_sensor.status}, {"ready", available_sensor.ready}, {"rate", available_sensor.rate}};
  }

  // Create the main JSON object using initializer list
  json json_msg = {{"robot_name", robot_name},
                   {"cpu_load", system_health_info->cpu_load},
                   {"free_ram", system_health_info->free_ram},
                   {"total_ram", system_health_info->total_ram},
                   {"free_hdd", system_health_info->free_hdd},
                   {"hw_api_rate", system_health_info->hw_api_rate},
                   {"control_manager_rate", system_health_info->control_manager_rate},
                   {"state_estimation_rate", system_health_info->state_estimation_rate},
                   {"gnss_uncertainty", system_health_info->gnss_uncertainty},
                   {"mag_strength", system_health_info->mag_strength},
                   {"mag_uncertainty", system_health_info->mag_uncertainty},
                   {"node_cpu_loads", node_cpu_loads},
                   {"available_sensors", available_sensors}};

  sendTelemetryJsonMessage("SystemHealthInfo", json_msg);
}

// --------------------------------------------------------------
// |                       helper methods                       |
// --------------------------------------------------------------

void IROCBridge::sendJsonMessage(const std::string &msg_type, json &json_msg) {
  const std::string url          = "/api/mission/" + msg_type;
  const std::string body         = json_msg.dump();
  const std::string content_type = "application/json";
  const auto res                 = http_client_->Post(url, body, content_type);

  if (res)
    RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, res->status << ": " << res->body);
  else
    RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Sent JSON message to " << url << ": " << body);

  return;
}

void IROCBridge::sendTelemetryJsonMessage(const std::string &type, json &json_msg) {

  json_msg["type"]    = type;
  std::string message = json_msg.dump();

  if (active_telemetry_connection_) {
    try {
      active_telemetry_connection_->send_text(message);
    }
    catch (const std::exception &e) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Websocket send_text failed, removing connection: " << e.what());
      active_telemetry_connection_ = nullptr;
    }
  }
}

template <typename ServiceType>
IROCBridge::result_t IROCBridge::callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request) {

  auto response = sc.callSync(request);

  if (response) {
    if (response.value()->success) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "  with response \"" << response.value()->message << "\".");
      return {true, response.value()->message};
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << "with response \"" << response.value()->message << "\".");
      return {false, response.value()->message};
    }
  } else {
    const std::string msg = std::string("Failed to call service ") + sc.getService() + ".";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, msg);
    return {false, msg};
  }
}

template <typename ServiceType>
IROCBridge::result_t IROCBridge::callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                                             const std::shared_ptr<typename ServiceType::Response> &response) {
  auto temp_response = sc.callSync(request);

  if (temp_response) {
    if (temp_response.value()->success) {
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << " with response \"" << temp_response.value()->message << "\".");
      *response = *(temp_response.value());
      return {true, temp_response.value()->message};
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "Called service " << sc.getService() << " with response \"" << temp_response.value()->message << "\".");
      *response = *(temp_response.value());
      return {false, temp_response.value()->message};
    }
  } else {
    const std::string msg = std::string("Failed to call service ") + sc.getService() + ".";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, msg);
    return {false, msg};
  }
}

void IROCBridge::routine_death_check() {
  // to enable graceful exit, the server needs to be stopped
  while (rclcpp::ok())
    clock_->sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(node_->get_logger(), "Stopping the HTTP server.");
  http_srv_.stop();

  RCLCPP_INFO(node_->get_logger(), "Stopping the HTTP client.");
  http_client_->stop();
}

IROCBridge::robot_handler_t *IROCBridge::findRobotHandler(const std::string &robot_name, robot_handlers_t &robot_handlers) {
  for (auto &rh : robot_handlers.handlers) {
    if (rh.robot_name == robot_name)
      return &rh;
  }

  return nullptr;
}

IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string> &robot_names, const std::string &command_type) {

  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  crow::status status_code = crow::status::ACCEPTED;
  ss << "Command: " << command_type << " Result: ";

  // Look up handler pointer-to-member
  auto it = trigger_command_handlers_.find(command_type);
  if (it == trigger_command_handlers_.end()) {
    ss << "Command type \"" << command_type << "\" not found\n";
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Command type \"" << command_type << "\" not found.");
    return {false, ss.str(), crow::status::NOT_FOUND};
  }

  auto handler_ptr = it->second;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Calling command \"" << command_type << "\".");

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  for (const auto &robot_name : robot_names) {
    auto *rh_ptr = findRobotHandler(robot_name, robot_handlers_);

    if (rh_ptr == nullptr) {
      ss << "Robot \"" << robot_name << "\" not found, skipping\n";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
      status_code   = crow::status::NOT_FOUND;
      continue;
    }

    // Access the service client using pointer-to-member
    auto &client    = rh_ptr->*handler_ptr;
    const auto resp = callService<std_srvs::srv::Trigger>(client, request);

    if (!resp.success) {
      ss << "Call for robot \"" << robot_name << "\" failed: " << resp.message << "\n";
      everything_ok = false;
      status_code   = crow::status::BAD_REQUEST;
    }
  }

  ss << "Successfully processed\n";
  return {everything_ok, ss.str(), status_code};
}

template <typename ServiceType>
IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string> &robot_names,
                                                      mrs_lib::ServiceClientHandler<ServiceType> robot_handler_t::*handler_member,
                                                      const std::shared_ptr<typename ServiceType::Request> &request) {

  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  crow::status status_code = crow::status::ACCEPTED;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Calling service action.");

  for (const auto &robot_name : robot_names) {
    auto *rh_ptr = findRobotHandler(robot_name, robot_handlers_);

    if (rh_ptr == nullptr) {
      ss << "Robot \"" << robot_name << "\" not found, skipping\n";
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
      status_code   = crow::status::NOT_FOUND;
      continue;
    }

    // Access the specific service client handler using pointer-to-member
    auto &client    = rh_ptr->*handler_member;
    const auto resp = callService<ServiceType>(client, request);

    if (!resp.success) {
      ss << "Call for robot \"" << robot_name << "\" was not successful: " << resp.message << "\n";
      everything_ok = false;
      status_code   = crow::status::BAD_REQUEST;
    }
  }

  ss << "Successfully processed\n";
  return {everything_ok, ss.str(), status_code};
}

template <typename Result>
json resultToJson(const std::shared_ptr<const Result> &result) {
  json robot_results = json::list();
  for (size_t i = 0; i < result->robot_results.size(); i++) {
    robot_results[i] = {{"robot", result->robot_results[i].name},
                        {"success", static_cast<bool>(result->robot_results[i].success)},
                        {"message", result->robot_results[i].message},
                        {"mission", json::list()}};
  }
  // Create the main JSON object
  json json_msg = {{"success", static_cast<bool>(result->success)}, {"message", result->message}, {"robot_data", robot_results}};
  return json_msg;
}

json missionGoalToJson(const iroc_fleet_manager::msg::MissionGoal &mission_goal) {
  json robots_data = json::list();

  auto robot_goals = mission_goal.robot_goals;

  for (size_t i = 0; i < robot_goals.size(); i++) {
    std::string robot_name = robot_goals.at(i).name;
    auto points            = robot_goals.at(i).points;
    int frame_id           = robot_goals.at(i).frame_id;
    int height_id          = robot_goals.at(i).height_id;

    // Extract the points
    json points_list = json::list();
    for (int j = 0; j < points.size(); j++) {
      mrs_msgs::msg::Reference reference = points.at(j).reference;
      json point     = {{"x", reference.position.x}, {"y", reference.position.y}, {"z", reference.position.z}, {"heading", reference.heading}};
      points_list[j] = std::move(point);
    }
    json mission    = {{"points", points_list}, {"frame_id", frame_id}, {"height_id", height_id}};
    json robot_data = {{"robot", robot_name}, {"success", true}, {"message", "Mission loaded successfully"}, {"mission", mission}};
    robots_data[i]  = std::move(robot_data);
  }

  json response = {
      {"success", true}, {"message", "Mission uploaded successfully"}, {"type", mission_goal.type}, {"uuid", mission_goal.uuid}, {"robot_data", robots_data}};


  // Temporary empty response until mission handler is done
  // json response = {
  // {"success", true}, {"message", "Mission uploaded successfully"}, {"type", mission_goal.type}, {"uuid", mission_goal.uuid}, {"robot_data", robots_data}};

  return response;
}

// --------------------------------------------------------------
// |                     REST API callbacks                     |
// --------------------------------------------------------------

/**
 * \brief Callback for the path endpoint. It parses the JSON message and publishes it to the corresponding robot handler.
 *
 * \param req The HTTP request containing the JSON message.
 * \return A response indicating the success or failure of the operation.
 */
crow::response IROCBridge::pathCallback(const crow::request &request) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing a path message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(request.body);
  if (!json_msg) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Bad json input: " << request.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + request.body + "\"}");
  }

  std::string robot_name = json_msg["robot_name"].s();
  std::scoped_lock lck(robot_handlers_.mtx);
  auto *rh_ptr = findRobotHandler(robot_name, robot_handlers_);
  if (!rh_ptr) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Robot \"" << robot_name << "\" not found. Ignoring.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Robot \"" + robot_name + "\" not found, ignoring\"}");
  }

  std::string frame_id = json_msg["frame_id"].s();
  if (frame_id.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Bad frame_id input: Expected a string.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad frame_id input: Expected a string.\"}");
  }

  std::vector<crow::json::rvalue> points = json_msg["points"].lo();
  if (points.empty()) {
    // ROS_WARN_STREAM("[IROCBridge]: Bad points input: Expected an array.");
    RCLCPP_WARN_STREAM(node_->get_logger(), "Bad points input: Expected an array.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad points input: Expected an array.\"}");
  }

  mrs_msgs::msg::Path msg_path;
  msg_path.points.reserve(points.size());
  msg_path.header.stamp    = clock_->now();
  msg_path.header.frame_id = frame_id;
  msg_path.fly_now         = true;
  msg_path.use_heading     = false;

  for (const auto &el : points) {
    mrs_msgs::msg::Reference ref;
    ref.position.x = el["x"].d();
    ref.position.y = el["y"].d();
    ref.position.z = el["z"].d();

    if (el.has("heading")) {
      ref.heading          = el["heading"].d();
      msg_path.use_heading = true;
    }

    msg_path.points.push_back(ref);
  }

  // Publish the path
  rh_ptr->pub_path.publish(msg_path);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Set a path with " << points.size() << " length.");
  return crow::response(crow::status::OK, "{\"message\": \"Set a path with " + std::to_string(points.size()) + " length.\"}");
}

/**
 * \brief Callback for the set safety border request. It receives a list of points and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setOriginCallback(const crow::request &request) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing a setOriginCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(request.body);
  if (!json_msg) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Bad json input: " << request.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + request.body + "\"}");
  }

  // The service supports latlon and UTM, but we we can at the moment support only latlon for IROC
  // TODO: It can be extended in the future to support UTM origin
  std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> service_request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
  service_request->header.frame_id                                             = "latlon_origin";
  service_request->header.stamp                                                = clock_->now();
  service_request->reference.position.x                                        = json_msg["x"].d();
  service_request->reference.position.y                                        = json_msg["y"].d();

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto &rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers
  const auto result = commandAction<mrs_msgs::srv::ReferenceStampedSrv>(robot_names, &robot_handler_t::sc_set_origin, service_request);

  if (result.success) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Set origin for " << robot_names.size() << " robots.");
    world_origin_.x = json_msg["x"].d();
    world_origin_.y = json_msg["y"].d();
  }

  return crow::response(result.status_code, result.message);
}

/**
 * \brief Callback to get the world origin.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getOriginCallback([[maybe_unused]] const crow::request &req) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing a getOriginCallback message ROS -> JSON.");

  std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Request> request   = std::make_shared<iroc_fleet_manager::srv::GetWorldOriginSrv::Request>();
  std::shared_ptr<iroc_fleet_manager::srv::GetWorldOriginSrv::Response> response = std::make_shared<iroc_fleet_manager::srv::GetWorldOriginSrv::Response>();

  const auto result = callService<iroc_fleet_manager::srv::GetWorldOriginSrv>(sc_get_world_origin_, request, response);

  json json_msg;
  if (!result.success) {
    json_msg["message"] = "Call was not successful with message: " + result.message;
    RCLCPP_WARN_STREAM(node_->get_logger(), json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {
    json_msg["message"] = response->message;
    json_msg["x"]       = response->origin_x;
    json_msg["y"]       = response->origin_y;
    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }
}

/**
 * \brief Callback for the set safety border request. It receives a list of points and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setSafetyBorderCallback(const crow::request &request) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing a setSafetyBorderCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(request.body);
  if (!json_msg) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Bad json input: " << request.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + request.body + "\"}");
  }

  // Get message properties
  int height_id                          = json_msg["height_id"].i();
  int max_z                              = json_msg["max_z"].i();
  int min_z                              = json_msg["min_z"].i();
  std::vector<crow::json::rvalue> points = json_msg["points"].lo();

  std::string horizontal_frame = "latlon_origin";
  std::string vertical_frame;

  std::map<int, std::string> height_id_map = {
      {0, "world_origin"},
      {1, "latlon_origin"},
  };

  auto it = height_id_map.find(height_id);
  if (it != height_id_map.end())
    vertical_frame = it->second;
  else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown height_id: " << height_id);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Unknown height_id field\"}");
  }

  std::vector<mrs_msgs::msg::Point2D> border_points;
  border_points.reserve(points.size());

  for (const auto &el : points) {
    mrs_msgs::msg::Point2D pt;
    pt.x = el["x"].d();
    pt.y = el["y"].d();

    border_points.push_back(pt);
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Border points size " << border_points.size());

  mrs_msgs::msg::SafetyBorder safety_border;
  safety_border.prism.points           = border_points;
  safety_border.prism.max_z            = max_z;
  safety_border.prism.min_z            = min_z;
  safety_border.prism.horizontal_frame = horizontal_frame;
  safety_border.prism.vertical_frame   = vertical_frame;

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto &rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers
  std::shared_ptr<mrs_msgs::srv::SetSafetyBorderSrv::Request> service_request = std::make_shared<mrs_msgs::srv::SetSafetyBorderSrv::Request>();
  service_request->prism                                                      = safety_border.prism;
  service_request->keep_obstacles                                             = false;

  const auto result = commandAction<mrs_msgs::srv::SetSafetyBorderSrv>(robot_names, &robot_handler_t::sc_set_safety_area, service_request);

  return crow::response(result.status_code, result.message);
}

/**
 * \brief Callback to get the safety area border.
 *
 * \param req Crow request
 * \return res Crow response
 */
int getFrameID(const std::string frame) {
  std::map<std::string, int> height_id_map = {
      {"world_origin", 0},
      {"latlon_origin", 1},
  };

  int id;
  auto it = height_id_map.find(frame);
  if (it != height_id_map.end())
    id = it->second;
  else
    id = -1;

  return id;
}

crow::response IROCBridge::getSafetyBorderCallback([[maybe_unused]] const crow::request &req) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing a getSafetyBorderCallback message ROS -> JSON.");

  std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request> request   = std::make_shared<iroc_fleet_manager::srv::GetSafetyBorderSrv::Request>();
  std::shared_ptr<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response> response = std::make_shared<iroc_fleet_manager::srv::GetSafetyBorderSrv::Response>();

  const auto result = callService<iroc_fleet_manager::srv::GetSafetyBorderSrv>(sc_get_safety_border_, request, response);

  json json_msg;
  if (!result.success) {
    json_msg["message"] = "Call was not successful with message: " + result.message;
    RCLCPP_WARN_STREAM(node_->get_logger(), json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {

    return crow::response(crow::status::NOT_IMPLEMENTED, "getSafetyBorderCallback not implemented yet");

    json_msg["message"]          = response->message;
    json points                  = json::list();
    auto vector_points           = response->border.points;
    double max_z                 = response->border.max_z;
    double min_z                 = response->border.min_z;
    std::string horizontal_frame = response->border.horizontal_frame;
    std::string vertical_frame   = response->border.vertical_frame;

    for (size_t i = 0; i < vector_points.size(); i++) {
      const auto point = vector_points.at(i);
      json point_json  = {{"x", point.x}, {"y", point.y}};
      points[i]        = std::move(point_json);
    }

    json json_msg = {{"message", "All robots in the fleet with the same safety border"},
                     {"points", points},
                     {"max_z", max_z},
                     {"min_z", min_z},
                     {"frame_id", getFrameID(horizontal_frame)},
                     {"height_id", getFrameID(vertical_frame)}};

    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }
}

/**
 * \brief Callback for the set obstacle request. It receives a list of obstacles for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setObstacleCallback(const crow::request &request) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Parsing a setObstacleCallback message JSON -> ROS.");
  crow::json::rvalue json_msg = crow::json::load(request.body);
  if (!json_msg)
    return crow::response(crow::status::BAD_REQUEST, "Failed to parse JSON" + request.body);

  // Check if obstacles array exists
  if (!json_msg.has("obstacles") || !json_msg["obstacles"]) {
    return crow::response(crow::status::BAD_REQUEST, "Missing obstacles array: " + request.body);
  }

  std::vector<crow::json::rvalue> obstacles = json_msg["obstacles"].lo();
  if (obstacles.empty()) {
    return crow::response(crow::status::BAD_REQUEST, "Empty obstacles array: " + request.body);
  }

  std::string horizontal_frame             = "latlon_origin";
  std::map<int, std::string> height_id_map = {
      {0, "world_origin"},
      {1, "latlon_origin"},
  };

  // Get robot names once (outside the loop)
  std::scoped_lock lck(robot_handlers_.mtx);
  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto &rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // Process each obstacle
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto &obstacle = obstacles[i];

    // Get obstacle properties
    if (!obstacle.has("height_id") || !obstacle.has("max_z") || !obstacle.has("min_z") || !obstacle.has("points")) {
      return crow::response(crow::status::BAD_REQUEST, "Missing required fields in obstacle " + std::to_string(i) + ": " + request.body);
    }

    int height_id                          = obstacle["height_id"].i();
    int max_z                              = obstacle["max_z"].i();
    int min_z                              = obstacle["min_z"].i();
    std::vector<crow::json::rvalue> points = obstacle["points"].lo();

    // Validate height_id
    std::string vertical_frame;
    auto it = height_id_map.find(height_id);
    if (it != height_id_map.end()) {
      vertical_frame = it->second;
    } else {
      return crow::response(crow::status::BAD_REQUEST, "Unknown height_id field in obstacle " + std::to_string(i) + ": " + request.body);
    }

    if (points.empty()) {
      return crow::response(crow::status::BAD_REQUEST, "Empty points array in obstacle " + std::to_string(i) + ": " + request.body);
    }

    // Process points
    std::vector<mrs_msgs::msg::Point2D> border_points;
    border_points.reserve(points.size());
    for (const auto &el : points) {
      if (!el.has("x") || !el.has("y")) {
        return crow::response(crow::status::BAD_REQUEST, "Missing x or y in point for obstacle " + std::to_string(i) + ": " + request.body);
      }
      mrs_msgs::msg::Point2D pt;
      pt.x = el["x"].d();
      pt.y = el["y"].d();
      border_points.push_back(pt);
    }

    // Logging
    RCLCPP_INFO_STREAM(node_->get_logger(), "Obstacle " << i << " border points size " << border_points.size());

    // Create obstacle request
    std::shared_ptr<mrs_msgs::srv::SetObstacleSrv::Request> service_request = std::make_shared<mrs_msgs::srv::SetObstacleSrv::Request>();
    service_request->prism.points                                           = border_points;
    service_request->prism.max_z                                            = max_z;
    service_request->prism.min_z                                            = min_z;
    service_request->prism.horizontal_frame                                 = horizontal_frame;
    service_request->prism.vertical_frame                                   = vertical_frame;

    // Call service for this obstacle
    const auto result = commandAction<mrs_msgs::srv::SetObstacleSrv>(robot_names, &robot_handler_t::sc_set_obstacle, service_request);

    // Check if the service call failed
    if (!result.success) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to set obstacle " << i << ": " << result.message);
      return crow::response(result.status_code, "Failed to set obstacle " + std::to_string(i) + ": " + result.message);
    }
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully set " << obstacles.size() << " obstacles");
  return crow::response(crow::status::OK, "Successfully set " + std::to_string(obstacles.size()) + " obstacles");
}

/**
 * \brief Callback to get the obstacles.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getObstaclesCallback([[maybe_unused]] const crow::request &req) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing a getObstaclesCallback message ROS -> JSON.");
  std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Request> request   = std::make_shared<iroc_fleet_manager::srv::GetObstaclesSrv::Request>();
  std::shared_ptr<iroc_fleet_manager::srv::GetObstaclesSrv::Response> response = std::make_shared<iroc_fleet_manager::srv::GetObstaclesSrv::Response>();

  const auto result = callService<iroc_fleet_manager::srv::GetObstaclesSrv>(sc_get_obstacles_, request, response);

  json json_msg;
  if (!result.success) {
    json_msg["message"] = "Call was not successful with message: " + result.message;
    RCLCPP_WARN_STREAM(node_->get_logger(), json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {

    return crow::response(crow::status::NOT_IMPLEMENTED, "{\"message\": \"getObstaclesCallback not implemented yet in fleet manager.\"}");

    json_msg["message"]      = response->message;
    auto obstacles           = response->obstacles;
    json obstacles_json_list = json::list();

    for (const auto &obstacles : response->obstacles) {
      json points_list = json::list();
      for (const auto &point : obstacles.points) {
        json point_json                 = {{"x", point.x}, {"y", point.y}};
        points_list[points_list.size()] = std::move(point_json);
      }
      json obstacle_json = {{"points", points_list},
                            {"max_z", obstacles.max_z},
                            {"min_z", obstacles.min_z},
                            {"frame_id", getFrameID(obstacles.horizontal_frame)},
                            {"height_id", getFrameID(obstacles.vertical_frame)}};
      // Add the obstacle JSON to the list of obstacles
      obstacles_json_list[obstacles_json_list.size()] = std::move(obstacle_json);
    }

    json json_msg = {{"message", "All robots in the fleet with the same obstacles"}, {"obstacles", obstacles_json_list}};
    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }
}

/**
 * \brief Callback to get the loaded mission.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getMissionCallback([[maybe_unused]] const crow::request &req) {

  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing a getMissionCallback message ROS -> JSON.");

  std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Request> request   = std::make_shared<iroc_fleet_manager::srv::GetMissionPointsSrv::Request>();
  std::shared_ptr<iroc_fleet_manager::srv::GetMissionPointsSrv::Response> response = std::make_shared<iroc_fleet_manager::srv::GetMissionPointsSrv::Response>();

  const auto result = callService<iroc_fleet_manager::srv::GetMissionPointsSrv>(sc_get_mission_data_, request, response);

  if (!result.success) {
    json error_response;
    error_response["message"]    = result.message;
    error_response["success"]    = false;
    error_response["robot_data"] = json::list();
    RCLCPP_WARN_STREAM(node_->get_logger(), error_response["message"].dump());
    return crow::response(crow::status::INTERNAL_SERVER_ERROR, error_response.dump());
  }

  auto mission_goal = response->mission_goal;

  auto json = missionGoalToJson(mission_goal);

  return crow::response(crow::status::OK, json.dump());
}

/**
 * \brief Callback for the mission upload request. Synchronously uploads/stages the mission
 * on all robots via the fleet manager upload service and returns per-robot results.
 *
 * Returns HTTP 200 with robot_results on full success, HTTP 400 on validation/staging failure,
 * HTTP 409 if a mission is already staged or executing.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::uploadMissionCallback(const crow::request &request) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing uploadMissionCallback: JSON -> upload service.");

  try {
    crow::json::rvalue json_msg = crow::json::load(request.body);

    if (!json_msg || !json_msg.has("type") || !json_msg.has("details"))
      return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad request: missing 'type' and/or 'details' keys\"}");

    std::string type = json_msg["type"].s();

    std::string uuid;
    if (!json_msg.has("uuid")) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Missing 'uuid' key in request, using empty string as default.");
      uuid = "";
    } else {
      uuid = json_msg["uuid"].s();
    }

    crow::json::wvalue details_wvalue(json_msg["details"]);
    std::string details = details_wvalue.dump();

    auto req_msg     = std::make_shared<iroc_fleet_manager::srv::UploadFleetMissionSrv::Request>();
    auto resp_msg    = std::make_shared<iroc_fleet_manager::srv::UploadFleetMissionSrv::Response>();
    req_msg->type    = type;
    req_msg->details = details;
    req_msg->uuid    = uuid;

    const auto call_result = callService<iroc_fleet_manager::srv::UploadFleetMissionSrv>(sc_upload_fleet_mission_, req_msg, resp_msg);

    if (!call_result.success) {
      json error_response;
      error_response["message"] = call_result.message;
      error_response["success"] = false;
      RCLCPP_WARN_STREAM(node_->get_logger(), "Upload mission service call failed: " << call_result.message);
      return crow::response(crow::status::INTERNAL_SERVER_ERROR, error_response.dump());
    }

    // Build robot_results JSON array
    json robot_results = json::list();
    for (size_t i = 0; i < resp_msg->robot_results.size(); i++) {
      robot_results[i] = {{"robot_name", resp_msg->robot_results[i].name},
                          {"success", static_cast<bool>(resp_msg->robot_results[i].success)},
                          {"message", resp_msg->robot_results[i].message}};
    }

    json response_json;
    response_json["success"]       = static_cast<bool>(resp_msg->success);
    response_json["message"]       = resp_msg->message;
    response_json["robot_results"] = std::move(robot_results);

    if (!resp_msg->success) {
      const auto &msg     = resp_msg->message;
      crow::status status = crow::status::BAD_REQUEST;
      if (msg.find("executing") != std::string::npos || msg.find("staged") != std::string::npos || msg.find("busy") != std::string::npos) {
        status = crow::status::CONFLICT;
      }
      RCLCPP_WARN_STREAM(node_->get_logger(), "Upload mission failed: " << resp_msg->message);
      return crow::response(status, response_json.dump());
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Upload mission successful: " << resp_msg->message);
    return crow::response(crow::status::OK, response_json.dump());
  }
  catch (const std::exception &e) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to parse JSON: " << e.what());
    json error_response;
    error_response["message"] = std::string("Failed to parse JSON: ") + e.what();
    return crow::response(crow::status::BAD_REQUEST, error_response.dump());
  }
}

/**
 * \brief Callback that changes the mission state of the fleet, either starting, stopping, or pausing it.
 * If starting and a mission is already active (but paused), it resumes via service call instead of sending a new action.
 * Initial start is sent as an action with empty goal, and fleet manager uses the pre-staged mission + auto-activates.
 * Start (resume)/stop/pause use the change_fleet_mission_state service, which calls sc_robot_activation on each robot in fleet manager.
 * Start works both for initial start and resume, stop and pause work for active missions. All return 409 if no mission is staged or executing.
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::changeFleetMissionStateCallback([[maybe_unused]] const crow::request &req, const std::string &type) {
  using FleetSrv = iroc_fleet_manager::srv::ChangeFleetMissionStateSrv;
  using FleetReq = FleetSrv::Request;

  // Translate URL string → typed enum at the HTTP boundary.
  static const std::unordered_map<std::string, uint8_t> kTypeMap = {
      {"start", FleetReq::TYPE_START},
      {"pause", FleetReq::TYPE_PAUSE},
      {"stop",  FleetReq::TYPE_STOP},
  };

  const auto it = kTypeMap.find(type);
  if (it == kTypeMap.end())
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Unknown operation: " + type + "\"}");

  const uint8_t op_type = it->second;

  // Helper to serialise per-robot results into the JSON response.
  auto buildResultsJson = [](const std::vector<iroc_mission_handler::msg::MissionResult> &results) {
    json arr = json::list();
    for (size_t i = 0; i < results.size(); i++) {
      arr[i] = {{"robot_name", results[i].name},
                {"success",    static_cast<bool>(results[i].success)},
                {"message",    results[i].message}};
    }
    return arr;
  };

  std::scoped_lock lck(robot_handlers_.mtx, mtx_current_goal_handle_);

  if (op_type == FleetReq::TYPE_START) {
    // If the fleet manager action is already live (mission running but paused),
    // resume via the change_fleet_mission_state service — fleet manager calls
    // sc_robot_activation on each robot.  Otherwise send a new action (initial
    // start) and fleet manager uses the pre-staged mission + auto-activates.
    if (current_goal_handle_) {
      const auto status   = current_goal_handle_->get_status();
      const bool is_alive = (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED || status == rclcpp_action::GoalStatus::STATUS_EXECUTING);
      if (is_alive) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Mission already active — resuming via service.");
        auto req_msg  = std::make_shared<FleetReq>();
        auto resp_msg = std::make_shared<FleetSrv::Response>();
        req_msg->type = FleetReq::TYPE_START;
        const auto call_result = callService<FleetSrv>(sc_change_fleet_mission_state_, req_msg, resp_msg);
        if (!call_result.success)
          return crow::response(crow::status::INTERNAL_SERVER_ERROR, "{\"message\": \"" + call_result.message + "\"}");
        json response_json;
        response_json["success"]       = static_cast<bool>(resp_msg->success);
        response_json["message"]       = resp_msg->message;
        response_json["robot_results"] = buildResultsJson(resp_msg->robot_results);
        const auto status_code = resp_msg->success ? crow::status::ACCEPTED : crow::status::INTERNAL_SERVER_ERROR;
        return crow::response(status_code, response_json.dump());
      }
    }

    // Initial start: send action with empty goal; block until accepted/rejected.
    if (!mission_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Action server not available for mission start.");
      json resp;
      resp["success"] = false;
      resp["message"] = "Action server not available. Check iroc_fleet_manager node.";
      return crow::response(crow::status::CONFLICT, resp.dump());
    }

    auto goal    = iroc_fleet_manager::action::ExecuteMission::Goal();
    goal.type    = "";
    goal.details = "";
    goal.uuid    = "";

    // Promise fulfilled by goal_response_callback with the accepted handle (nullptr = rejected).
    auto goal_response_promise = std::make_shared<std::promise<MissionGoalHandle::SharedPtr>>();
    auto goal_response_future  = goal_response_promise->get_future();

    auto send_goal_options = rclcpp_action::Client<Mission>::SendGoalOptions();

    send_goal_options.goal_response_callback = [goal_response_promise](MissionGoalHandle::SharedPtr goal_handle) {
      goal_response_promise->set_value(goal_handle);
    };

    send_goal_options.result_callback = [this](const MissionGoalHandle::WrappedResult &result) { this->missionDoneCallback(result); };

    send_goal_options.feedback_callback = [this](MissionGoalHandle::SharedPtr, const Mission::Feedback::ConstSharedPtr feedback) {
      this->missionFeedbackCallback(feedback);
    };

    mission_client_->async_send_goal(goal, send_goal_options);

    // Wait for fleet manager to accept or reject (no lock needed in callback).
    if (goal_response_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Timed out waiting for fleet manager goal response.");
      json resp;
      resp["success"] = false;
      resp["message"] = "Timed out waiting for fleet manager to accept mission start.";
      return crow::response(crow::status::INTERNAL_SERVER_ERROR, resp.dump());
    }

    const auto accepted_handle = goal_response_future.get();
    if (!accepted_handle) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Mission start rejected by fleet manager.");
      json resp;
      resp["success"] = false;
      resp["message"] = "Mission start rejected by fleet manager. Ensure a mission is uploaded first.";
      return crow::response(crow::status::CONFLICT, resp.dump());
    }

    current_goal_handle_ = accepted_handle;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Mission start accepted by fleet manager.");
    json resp;
    resp["success"] = true;
    resp["message"] = "Mission started.";
    return crow::response(crow::status::ACCEPTED, resp.dump());
  }

  // pause / stop: forward to fleet manager service and return structured response
  auto req_msg  = std::make_shared<FleetReq>();
  auto resp_msg = std::make_shared<FleetSrv::Response>();
  req_msg->type = op_type;
  const auto call_result = callService<FleetSrv>(sc_change_fleet_mission_state_, req_msg, resp_msg);
  if (!call_result.success)
    return crow::response(crow::status::INTERNAL_SERVER_ERROR, "{\"message\": \"" + call_result.message + "\"}");
  json response_json;
  response_json["success"]       = static_cast<bool>(resp_msg->success);
  response_json["message"]       = resp_msg->message;
  response_json["robot_results"] = buildResultsJson(resp_msg->robot_results);
  const auto status_code = resp_msg->success ? crow::status::ACCEPTED : crow::status::INTERNAL_SERVER_ERROR;
  return crow::response(status_code, response_json.dump());
}

/**
 * \brief Callback that changes the mission state of a specific robot, either starting, stopping, or pausing it.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::changeRobotMissionStateCallback([[maybe_unused]] const crow::request &req, const std::string &robot_name, const std::string &type) {
  using RobotReq = iroc_fleet_manager::srv::ChangeRobotMissionStateSrv::Request;

  // Translate URL string → typed enum at the HTTP boundary.
  static const std::unordered_map<std::string, uint8_t> kTypeMap = {
      {"start", RobotReq::TYPE_START},
      {"pause", RobotReq::TYPE_PAUSE},
      {"stop",  RobotReq::TYPE_STOP},
  };

  const auto it = kTypeMap.find(type);
  if (it == kTypeMap.end())
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Unknown operation: " + type + "\"}");

  std::scoped_lock lck(robot_handlers_.mtx);

  if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(), [&robot_name](const auto &rh) { return rh.robot_name == robot_name; }))
    return crow::response(crow::status::NOT_FOUND, "{\"message\": \"Robot not found: " + robot_name + "\"}");

  json json_msg;
  auto request        = std::make_shared<RobotReq>();
  request->robot_name = robot_name;
  request->type       = it->second;

  const auto resp = callService<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv>(sc_change_robot_mission_state_, request);

  if (!resp.success) {
    json_msg["message"] = "Call was not successful: " + resp.message;
    RCLCPP_WARN_STREAM(node_->get_logger(), json_msg["message"].dump());
    return crow::response(crow::status::INTERNAL_SERVER_ERROR, json_msg.dump());
  }

  json_msg["message"] = "Call successful";
  return crow::response(crow::status::ACCEPTED, json_msg.dump());
}

/*!
 * \brief Callback that sends a `hover` command to the robots specified in the request body.
 * \param req Crow request
 *
 * \return res Crow response
 */
crow::response IROCBridge::commandCallback([[maybe_unused]] const crow::request &req, const std::string &command_type, std::optional<std::string> robot_name) {
  std::scoped_lock lck(robot_handlers_.mtx);
  std::vector<std::string> robot_names;

  if (robot_name.has_value()) {
    robot_names.push_back(robot_name.value());
  } else {
    robot_names.reserve(robot_handlers_.handlers.size());
    for (const auto &rh : robot_handlers_.handlers) {
      robot_names.push_back(rh.robot_name);
    }
  }

  const auto result = commandAction(robot_names, command_type);
  return crow::response(result.status_code, result.message);
}

/**
 * \brief Callback that returns a list of available robot names in the fleet based on `robot_handlers_` vector.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::availableRobotsCallback([[maybe_unused]] const crow::request &req) {
  if (robot_handlers_.handlers.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "No robots available in the fleet.");
    return crow::response(crow::status::NO_CONTENT, "{\"message\": \"No robots available in the fleet.\"}");
  }

  json robots = json::list();
  for (size_t i = 0; i < robot_handlers_.handlers.size(); i++) {
    if (!robot_handlers_.handlers[i].sh_general_robot_info.hasMsg()) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Robot handler for robot " << robot_handlers_.handlers[i].robot_name << " does not have general robot info.");
      continue;
    }

    robots[robots.size()] = {{"name", robot_handlers_.handlers[i].sh_general_robot_info.getMsg()->robot_name},
                             {"type", robot_handlers_.handlers[i].sh_general_robot_info.getMsg()->robot_type}};
  }

  return crow::response(crow::status::ACCEPTED, robots.dump());
}

/**
 * \brief Callback that is called when a websocket message is received, it checks the command and sends it to the robot
 * through a
 * [`mrs_msgs::VelocityReferenceStampedSrv`](https://ctu-mrs.github.io/mrs_msgs/srv/VelocityReferenceStampedSrv.html)
 * service.
 *
 * \param conn Crow Websocket connection
 * \param data Data received from the websocket.
 * \param is_binary Flag if the data is binary
 * \return void
 *
 * \note The velocity values are normalized between -1.0 and 1.0, where:
 * - For `x`, `y`, `z`: Values are multiplied by max_linear_speed_ ros param
 * - For `heading`: Values are multiplied by max_heading_rate_ ros param
 *
 * **Command Structure:**
 * - `{"command": "message", "data": "Hello, world!"}`
 * - `{"command": "move", "robot_name": "robot_1", "data": {"x": 0.5, "y": 0.5, "z": 0.5, "heading": 0.5}}`
 *
 * **Response Messages:**
 * - For successful `message` command: `{"status": "Ok, received message"}`
 * - For successful `move` command: `{"ok": true, "message": "Movement command sent"}`
 * - For JSON parsing errors: `{"error": "Failed to parse JSON"}`
 * - For missing robot name: `{"error": "Missing robot_id"}`
 * - For invalid robot name: `{"error": "Robot not found"}`
 * - For unknown commands: `{"ok": false, "message": "Unknown command"}`
 * - For movement command failures: `{"ok": false, "message": "Failed to send movement command: [error details]"}`
 */
void IROCBridge::remoteControlCallback(crow::websocket::connection &conn, const std::string &data, bool is_binary) {
  // Convert and check if the received data is a valid JSON
  crow::json::rvalue json_data = crow::json::load(data);
  if (!json_data || !json_data.has("command") || !json_data.has("data")) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to parse JSON from websocket message: " << data);
    conn.send_text("{\"error\": \"Failed to parse JSON or missing 'command'/'data'\"}");
    return;
  }

  json json_response;
  std::string command = json_data["command"].s();
  if (command == "message") {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Received message from " << conn.get_remote_ip() << ": " << json_data["data"].s());
    conn.send_text("{\"status\": \"Ok, received message\"}");
  } else if (command == "move") {
    std::scoped_lock lck(robot_handlers_.mtx);

    // Robot id validation
    if (!json_data.has("robot_name")) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Missing robot_name in websocket message: " << data);
      conn.send_text("{\"error\": \"Missing robot_id\"}");
      return;
    }

    std::string robot_name = json_data["robot_name"].s();
    if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(),
                     [robot_name](const robot_handler_t &rh) { return rh.robot_name == robot_name; })) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Robot \"" << robot_name << "\" not found in websocket message: " << data);
      conn.send_text("{\"error\": \"Robot not found\"}");
      return;
    }

    crow::json::rvalue movement_data = json_data["data"];

    std::shared_ptr<mrs_msgs::srv::VelocityReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::VelocityReferenceStampedSrv::Request>();

    request->reference.header.frame_id = "fcu_untilted";

    request->reference.reference.velocity.x       = movement_data["x"].d() * max_linear_speed_;
    request->reference.reference.velocity.y       = movement_data["y"].d() * max_linear_speed_;
    request->reference.reference.velocity.z       = movement_data["z"].d() * max_linear_speed_;
    request->reference.reference.heading_rate     = movement_data["heading"].d() * max_heading_rate_;
    request->reference.reference.use_heading_rate = true;

    auto *robot_handler_ptr = findRobotHandler(robot_name, robot_handlers_);
    auto res                = callService<mrs_msgs::srv::VelocityReferenceStampedSrv>(robot_handler_ptr->sc_velocity_reference, request);

    if (res.success) {
      json_response["ok"]      = true;
      json_response["message"] = "Movement command sent";
    } else {
      json_response["ok"]      = false;
      json_response["message"] = "Failed to send movement command: " + res.message;
    }
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown command in websocket message: " << command);
    json_response["ok"]      = false;
    json_response["message"] = "Unknown command";
  }
  conn.send_text(json_response.dump());
}

} // namespace iroc_bridge
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(iroc_bridge::IROCBridge)
