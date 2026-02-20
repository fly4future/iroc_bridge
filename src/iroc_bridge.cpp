/* includes //{ */

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>

#include <httplib/httplib.h>

#include "crow.h"
#include "crow/middlewares/cors.h"

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <mrs_msgs/Path.h>

#include <mrs_msgs/String.h>
#include <iroc_fleet_manager/ChangeRobotMissionStateSrv.h>
#include <iroc_fleet_manager/GetWorldOriginSrv.h>
#include <iroc_fleet_manager/GetSafetyBorderSrv.h>
#include <iroc_fleet_manager/GetObstaclesSrv.h>
#include <iroc_fleet_manager/GetMissionPointsSrv.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ReferenceStampedSrvRequest.h>
#include <mrs_msgs/ReferenceStampedSrvResponse.h>
#include <mrs_msgs/ReferenceStamped.h>

#include <mrs_msgs/SetSafetyBorderSrv.h>
#include <mrs_msgs/SetSafetyBorderSrvRequest.h>
#include <mrs_msgs/SetSafetyBorderSrvResponse.h>

#include <mrs_msgs/VelocityReferenceStampedSrv.h>

#include <mrs_msgs/SetObstacleSrv.h>
#include <mrs_msgs/SetObstacleSrvRequest.h>
#include <mrs_msgs/SetObstacleSrvResponse.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>
#include <mrs_robot_diagnostics/SensorInfo.h>
#include <mrs_robot_diagnostics/enums/robot_type.h>

#include <iroc_fleet_manager/IROCFleetManagerAction.h>
#include <iroc_fleet_manager/IROCFleetMissionGoal.h>

#include <unistd.h>
#include <iostream>

//}

namespace iroc_bridge {

using json = crow::json::wvalue;

using vec3_t = Eigen::Vector3d;
using vec4_t = Eigen::Vector4d;

using namespace actionlib;

using FleetManagerActionClient = SimpleActionClient<iroc_fleet_manager::IROCFleetManagerAction>;
// Waypoint mission goal
typedef iroc_fleet_manager::IROCFleetManagerGoal FleetManagerGoal;
typedef mrs_robot_diagnostics::robot_type_t robot_type_t;

/* class IROCBridge //{ */
class IROCBridge : public nodelet::Nodelet {
 public:
  virtual void onInit();

 private:
  ros::NodeHandle nh_;

  // | ---------------------- HTTP REST API --------------------- |
  std::thread th_http_srv_;
  crow::App<crow::CORSHandler> http_srv_;

  std::unique_ptr<httplib::Client> http_client_;

  struct result_t {
    bool success;
    std::string message;
  };

  struct action_result_t {
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

  enum class Change_SvC_T { FleetWaypoint, RobotWaypoint, FleetCoverage, RobotCoverage, RobotAutonomyTest };

  std::map<std::string, CommandType> command_type_map_ = {
      {"takeoff", CommandType::Takeoff},
      {"land", CommandType::Land},
      {"hover", CommandType::Hover},
      {"home", CommandType::Home},
      {"set_origin", CommandType::Set_Origin},
      {"set_safety_border", CommandType::Set_SafetyBorder},
      {"set_obstacle", CommandType::Set_Obstacle},
  };

  // | ---------------------- ROS parameters ------------------ |
  double max_linear_speed_;
  double max_heading_rate_;

  // | ---------------------- ROS subscribers --------------------- |

  // Fleet manager feedback subscriber
  mrs_lib::SubscribeHandler<iroc_fleet_manager::IROCFleetManagerActionFeedback> sch_fleet_manager_feedback_; 

  struct robot_handler_t {
    std::string robot_name;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo> sh_general_robot_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo> sh_state_estimation_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo> sh_control_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo> sh_collision_avoidance_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo> sh_uav_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo> sh_system_health_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SensorInfo> sh_sensor_info;

    ros::ServiceClient sc_takeoff;
    ros::ServiceClient sc_land;
    ros::ServiceClient sc_hover;
    ros::ServiceClient sc_land_home;
    ros::ServiceClient sc_set_origin;
    ros::ServiceClient sc_set_safety_area;
    ros::ServiceClient sc_set_obstacle;
    ros::ServiceClient sc_velocity_reference;

    ros::Publisher pub_path;
  };

  struct robot_handlers_t {
    std::recursive_mutex mtx;
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void timerMain(const ros::TimerEvent& event);

  // | ----------------------- ROS Clients ----------------------- |
  ros::ServiceClient sc_change_fleet_mission_state_;
  ros::ServiceClient sc_change_robot_mission_state_;
  ros::ServiceClient sc_get_world_origin_;
  ros::ServiceClient sc_get_safety_border_;
  ros::ServiceClient sc_get_obstacles_;
  ros::ServiceClient sc_get_mission_data_;

  // | ----------------- action client callbacks ---------------- |

  // Mission callbacks
  void missionActiveCallback();
  template <typename Result>
  void missionDoneCallback(const SimpleClientGoalState& state, const boost::shared_ptr<const Result>& result);
  template <typename Feedback>
  void missionFeedbackCallback(const boost::shared_ptr<const Feedback>& feedback);
  void missionFeedbackCallback(iroc_fleet_manager::IROCFleetManagerActionFeedback::ConstPtr msg);

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string& robot_name);
  void parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string& robot_name);
  void parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name);
  void parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name);
  void parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name);
  void parseSensorInfo(mrs_robot_diagnostics::SensorInfo::ConstPtr sensor_info, const std::string& robot_name);
  void parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr uav_info, const std::string& robot_name);

  void sendJsonMessage(const std::string& msg_type, json& json_msg);
  void sendTelemetryJsonMessage(const std::string& type, json& json_msg);
  robot_handler_t* findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers);

  ros::ServiceClient* getServiceClient(IROCBridge::robot_handler_t* rh_ptr, const IROCBridge::CommandType command_type);
  ros::ServiceClient getServiceClient(const IROCBridge::Change_SvC_T service_type);

  action_result_t commandAction(const std::vector<std::string>& robot_names, const std::string& command_type);

  template <typename Svc_T>
  action_result_t commandAction(const std::vector<std::string>& robot_names, const std::string& command_type, typename Svc_T::Request req);

  // REST API callbacks
  crow::response pathCallback(const crow::request& req);
  crow::response setOriginCallback(const crow::request& req);
  crow::response getOriginCallback(const crow::request& req);
  crow::response setSafetyBorderCallback(const crow::request& req);
  crow::response getSafetyBorderCallback(const crow::request& req);
  crow::response setObstacleCallback(const crow::request& req);
  crow::response getObstaclesCallback(const crow::request& req);
  crow::response missionCallback(const crow::request& req);
  crow::response getMissionCallback(const crow::request& req);

  crow::response changeFleetMissionStateCallback(const crow::request& req, const std::string& type);
  crow::response changeRobotMissionStateCallback(const crow::request& req, const std::string& robot_name, const std::string& type);

  crow::response availableRobotsCallback(const crow::request& req);
  crow::response commandCallback(const crow::request& req, const std::string& command_type, std::optional<std::string> robot_name);

  // Websocket callbacks
  void remoteControlCallback(crow::websocket::connection& conn, const std::string& data, bool is_binary);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req,  typename Svc_T::Response &res);


  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

  std::thread th_death_check_;
  std::thread th_telemetry_check_;
  void routine_death_check();
  crow::websocket::connection* active_telemetry_connection_ = nullptr;
  std::mutex mtx_telemetry_connections_;

  std::unique_ptr<FleetManagerActionClient> fleet_manager_action_client_;

  // Latlon origin
  mrs_msgs::Point2D world_origin_;
};
//}

/* onInit() //{ */
void IROCBridge::onInit() {
  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  std::vector<char> hostname(1024);

  if (gethostname(hostname.data(), hostname.size()) == 0) {
    std::cout << "Hostname: " << hostname.data() << std::endl;
  } else {
    std::cerr << "Failed to get hostname" << std::endl;
  }

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  /* load parameters */
  mrs_lib::ParamLoader param_loader(nh_, "IROCBridge");

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

  // Arguments given in launchfile
  const auto url = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");
  
  const auto main_timer_rate = param_loader.loadParam2<double>("iroc_bridge/main_timer_rate");
  const auto _http_server_threads_ = param_loader.loadParam2<double>("iroc_bridge/http_server_threads");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("iroc_bridge/no_message_timeout");

  max_linear_speed_ = param_loader.loadParam2<double>("iroc_bridge/remote_control_limits/max_linear_speed");
  max_heading_rate_ = param_loader.loadParam2<double>("iroc_bridge/remote_control_limits/max_heading_rate");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------- HTTP REST API callbacks ---------------- |
  // Client
  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  // Server
  // Do we need this (set_path)?
  CROW_ROUTE(http_srv_, "/set_path").methods(crow::HTTPMethod::Post)([this](const crow::request& req) { return pathCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/world-origin").methods(crow::HTTPMethod::Post)([this](const crow::request& req) { return setOriginCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/borders").methods(crow::HTTPMethod::Post)([this](const crow::request& req) { return setSafetyBorderCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/obstacles").methods(crow::HTTPMethod::Post)([this](const crow::request& req) { return setObstacleCallback(req); });
  CROW_ROUTE(http_srv_, "/mission").methods(crow::HTTPMethod::Post)([this](const crow::request& req) { return missionCallback(req); });

  // Getters
  CROW_ROUTE(http_srv_, "/safety-area/world-origin").methods(crow::HTTPMethod::Get)([this](const crow::request& req) { return getOriginCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/borders").methods(crow::HTTPMethod::Get)([this](const crow::request& req) { return getSafetyBorderCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/obstacles").methods(crow::HTTPMethod::Get)([this](const crow::request& req) { return getObstaclesCallback(req); });
  CROW_ROUTE(http_srv_, "/mission").methods(crow::HTTPMethod::Get)([this](const crow::request& req) { return getMissionCallback(req); });

  // Missions
  // TODO: CROW_REGEX_ROUTE(http_srv_, R"(/fleet/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/mission/<string>").methods(crow::HTTPMethod::Post)([this](const crow::request& req, const std::string& type) {
    return changeFleetMissionStateCallback(req, type);
  });
  // TODO: CROW_REGEX_ROUTE(http_srv_, R"(/robots/(\w+)/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/robots/<string>/mission/<string>")
      .methods(crow::HTTPMethod::Post)([this](const crow::request& req, const std::string& robot_name, const std::string& type) {
        return changeRobotMissionStateCallback(req, robot_name, type);
      });

  // Available robots endpoint
  CROW_ROUTE(http_srv_, "/robots").methods(crow::HTTPMethod::Get)([this](const crow::request& req) { return availableRobotsCallback(req); });

  // Command endpoints with robot name in the path (land, takeoff, hover, home)
  CROW_ROUTE(http_srv_, "/robots/<string>/<string>")
      .methods(crow::HTTPMethod::Post)([this](const crow::request& req, const std::string& robot_name, const std::string& command_type) {
        return commandCallback(req, command_type, robot_name);
      });

  // Command endpoint for all robots (land, takeoff, hover, home)
  CROW_ROUTE(http_srv_, "/robots/<string>").methods(crow::HTTPMethod::Post)([this](const crow::request& req, const std::string& command_type) {
    return commandCallback(req, command_type, std::nullopt);
  });

  // Remote control websocket
  CROW_WEBSOCKET_ROUTE(http_srv_, "/rc")
      .onopen([&](crow::websocket::connection& conn) {
        ROS_INFO_STREAM("[IROCBridge]: New remote control websocket connection: " << &conn);
        ROS_INFO_STREAM("[IROCBridge]: New remote control websocket connection: " << conn.get_remote_ip());
      })
      .onclose([&](crow::websocket::connection& conn, const std::string& reason, int code) {
        ROS_INFO_STREAM("[IROCBridge]: Websocket connection " << conn.get_remote_ip() << " closed: " << reason);
        ROS_INFO_STREAM("[IROCBridge]: Websocket connection " << &conn << " closed: " << reason);
      })
      .onmessage([this](crow::websocket::connection& conn, const std::string& data, bool is_binary) { return remoteControlCallback(conn, data, is_binary); });

  // Telemetry websocket
  CROW_WEBSOCKET_ROUTE(http_srv_, "/telemetry")
      .onopen([&](crow::websocket::connection& conn) {
        ROS_INFO_STREAM("[IROCBridge]: New telemetry websocket connection: " << &conn);
        ROS_INFO_STREAM("[IROCBridge]: New telemetry websocket connection: " << conn.get_remote_ip());
        std::scoped_lock lock(mtx_telemetry_connections_);
        active_telemetry_connection_ = &conn;
      })
      .onclose([&](crow::websocket::connection& conn, const std::string& reason, int code) {
        ROS_INFO_STREAM("[IROCBridge]: Websocket connection " << conn.get_remote_ip() << " closed: " << reason);
        ROS_INFO_STREAM("[IROCBridge]: Websocket connection " << &conn << " closed: " << reason);
        std::scoped_lock lock(mtx_telemetry_connections_);
        if (active_telemetry_connection_ == &conn) {
          active_telemetry_connection_ = nullptr;
        }
      });

  th_http_srv_ = std::thread([&]() { http_srv_.loglevel(crow::LogLevel::ERROR).port(server_port).concurrency(_http_server_threads_).run(); });
  ROS_INFO("[IROCBridge]: Threads using %d ", http_srv_.concurrency());
  th_http_srv_.detach();

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh = nh_;
  shopts.node_name = "IROCBridge";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.queue_size = 10;
  shopts.transport_hints = ros::TransportHints().tcpNoDelay();

  // populate the robot handlers vector
  {
    std::scoped_lock lck(robot_handlers_.mtx);

    robot_handlers_.handlers.reserve(filtered_robot_names.size());
    for (const auto& robot_name : filtered_robot_names) {
      robot_handler_t robot_handler;
      // To share with fleet manager and planners
      robot_handler.robot_name = robot_name;
      // Saving only the robot names to fill up the rest during the parsing of the messages
      const std::string general_robot_info_topic_name = "/" + robot_name + nh_.resolveName("in/general_robot_info");
      robot_handler.sh_general_robot_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo>(shopts, general_robot_info_topic_name);

      const std::string state_estimation_info_topic_name = "/" + robot_name + nh_.resolveName("in/state_estimation_info");
      robot_handler.sh_state_estimation_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo>(shopts, state_estimation_info_topic_name);

      const std::string control_info_topic_name = "/" + robot_name + nh_.resolveName("in/control_info");
      robot_handler.sh_control_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo>(shopts, control_info_topic_name);

      const std::string collision_avoidance_info_topic_name = "/" + robot_name + nh_.resolveName("in/collision_avoidance_info");
      robot_handler.sh_collision_avoidance_info =
          mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo>(shopts, collision_avoidance_info_topic_name);

      const std::string uav_info_topic_name = "/" + robot_name + nh_.resolveName("in/uav_info");
      robot_handler.sh_uav_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo>(shopts, uav_info_topic_name);

      const std::string system_health_info_topic_name = "/" + robot_name + nh_.resolveName("in/system_health_info");
      robot_handler.sh_system_health_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo>(shopts, system_health_info_topic_name);

      const std::string sensor_info_topic_name = "/" + robot_name + nh_.resolveName("in/sensor_info");
      robot_handler.sh_sensor_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SensorInfo>(shopts, sensor_info_topic_name);

      robot_handler.sc_takeoff = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/takeoff"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/takeoff\' -> \'%s\'", robot_handler.sc_takeoff.getService().c_str());

      robot_handler.sc_hover = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/hover"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/hover\' -> \'%s\'", robot_handler.sc_hover.getService().c_str());

      robot_handler.sc_land = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/land"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land\' -> \'%s\'", robot_handler.sc_land.getService().c_str());

      robot_handler.sc_land_home = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/land_home"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land_home\' -> \'%s\'", robot_handler.sc_land_home.getService().c_str());

      robot_handler.sc_set_origin = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + robot_name + nh_.resolveName("svc/set_origin"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/set_origin\' -> \'%s\'", robot_handler.sc_set_origin.getService().c_str());

      robot_handler.sc_set_safety_area = nh_.serviceClient<mrs_msgs::SetSafetyBorderSrv>("/" + robot_name + nh_.resolveName("svc/set_safety_area"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/set_safety_area\' -> \'%s\'", robot_handler.sc_set_safety_area.getService().c_str());

      robot_handler.sc_set_obstacle = nh_.serviceClient<mrs_msgs::SetObstacleSrv>("/" + robot_name + nh_.resolveName("svc/set_obstacle"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/set_obstacle\' -> \'%s\'", robot_handler.sc_set_obstacle.getService().c_str());

      robot_handler.sc_velocity_reference =
          nh_.serviceClient<mrs_msgs::VelocityReferenceStampedSrv>("/" + robot_name + nh_.resolveName("svc/velocity_reference"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/velocity_reference\' -> \'%s\'", robot_handler.sc_velocity_reference.getService().c_str());

      // | ----------------------- publishers ----------------------- |
      robot_handler.pub_path = nh_.advertise<mrs_msgs::Path>("/" + robot_name + nh_.resolveName("out/path"), 2);
      ROS_INFO("[IROCBridge]: Created publisher on topic \'out/path\' -> \'%s\'", robot_handler.pub_path.getTopic().c_str());

      // move is necessary because copy construction of the subscribe handlers is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }

  shopts.no_message_timeout = mrs_lib::no_timeout;
  sch_fleet_manager_feedback_ =
      mrs_lib::SubscribeHandler<iroc_fleet_manager::IROCFleetManagerActionFeedback>(shopts, "in/fleet_manager_feedback");

  sc_change_fleet_mission_state_ = nh_.serviceClient<mrs_msgs::String>(nh_.resolveName("svc/change_fleet_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_fleet_mission_state\' -> \'%s\'",
           sc_change_fleet_mission_state_.getService().c_str());

  sc_change_robot_mission_state_ = nh_.serviceClient<iroc_fleet_manager::ChangeRobotMissionStateSrv>(nh_.resolveName("svc/change_robot_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_robot_mission_state\' -> \'%s\'",
           sc_change_robot_mission_state_.getService().c_str());

  sc_get_world_origin_ = nh_.serviceClient<iroc_fleet_manager::GetWorldOriginSrv>(nh_.resolveName("svc/get_world_origin"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/get_world_origin\' -> \'%s\'",
           sc_get_world_origin_.getService().c_str());

  sc_get_safety_border_ = nh_.serviceClient<iroc_fleet_manager::GetSafetyBorderSrv>(nh_.resolveName("svc/get_safety_border"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/get_safety_border\' -> \'%s\'",
           sc_get_safety_border_.getService().c_str());

  sc_get_obstacles_ = nh_.serviceClient<iroc_fleet_manager::GetObstaclesSrv>(nh_.resolveName("svc/get_obstacles"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/get_obstacles\' -> \'%s\'",
           sc_get_obstacles_.getService().c_str());

  sc_get_mission_data_ = nh_.serviceClient<iroc_fleet_manager::GetMissionPointsSrv>(nh_.resolveName("svc/get_mission_data"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/get_mission_data\' -> \'%s\'",
           sc_get_mission_data_.getService().c_str());

  /* // | --------------------- action clients --------------------- | */

  // Mission action client
  const std::string action_client_topic = nh_.resolveName("ac/mission");
  fleet_manager_action_client_ = std::make_unique<FleetManagerActionClient>(action_client_topic, false);
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/mission\' -> \'%s\'", action_client_topic.c_str());

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
void IROCBridge::timerMain([[maybe_unused]] const ros::TimerEvent& event) {
  std::scoped_lock lck(robot_handlers_.mtx);

  // Parsing the messages to send in telemetry
  for (auto& rh : robot_handlers_.handlers) {
    const auto& robot_name = rh.robot_name;

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

    if (sch_fleet_manager_feedback_.newMsg()) {
      missionFeedbackCallback(sch_fleet_manager_feedback_.getMsg());
    }
  }
}
//}

// --------------------------------------------------------------
// |                  action client callbacks                   |
// --------------------------------------------------------------

// Mission callbacks
/* missionActiveCallback //{ */
void IROCBridge::missionActiveCallback() {
  ROS_INFO_STREAM("[IROCBridge]: Mission Action server for robots: ");
}
//}

/* missionDoneCallback //{ */
template <typename Result>
void IROCBridge::missionDoneCallback(const SimpleClientGoalState& state, const boost::shared_ptr<const Result>& result) {
  if (result == nullptr) {
    ROS_WARN("[IROCBridge]: Probably fleet_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was "
             "restarted need to upload a new mission!");

    // Create JSON with Crow
    json json_msg = {
        {"success", false},
        {"message", "Fleet manager died in ongoing mission"},
        {"robot_results", "Fleet manager died in ongoing mission"},
    };
    sendTelemetryJsonMessage("results", json_msg);

  } else {
    if (result->success) {
      ROS_INFO_STREAM("[IROCBridge]: Mission Action server finished with state: \"" << state.toString() << "\"");
    } else {
      ROS_INFO_STREAM("[IROCBridge]: Mission Action server finished with state: \"" << state.toString() << "\"");
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
}
//}

/* missionFeedbackCallback //{ */
template <typename Feedback>
void IROCBridge::missionFeedbackCallback(const boost::shared_ptr<const Feedback>& feedback) {
  auto robot_feedbacks = feedback->info.robot_feedbacks;

  // Create a list for robot feedback
  json json_msgs = json::list();

  // Collect each robot feedback and create a json for each
  for (size_t i = 0; i < robot_feedbacks.size(); i++) {
    const auto& rfb = robot_feedbacks[i];

    json robot_json = {{"robot_name", rfb.name},
                       {"message", rfb.message},
                       {"mission_progress", rfb.mission_progress},
                       {"current_goal", rfb.goal_idx},
                       {"distance_to_goal", rfb.distance_to_closest_goal},
                       {"goal_estimated_arrival_time", rfb.goal_estimated_arrival_time},
                       {"goal_progress", rfb.goal_progress},
                       {"distance_to_finish", rfb.distance_to_finish},
                       {"finish_estimated_arrival_time", rfb.finish_estimated_arrival_time}};

    ROS_DEBUG_STREAM("[IROCBridge]: Mission feedback for robot: "
                     << rfb.name << ", message: " << rfb.message << ", progress: " << rfb.mission_progress << ", current goal: " << rfb.goal_idx
                     << ", distance to goal: " << rfb.distance_to_closest_goal << ", goal estimated arrival time: " << rfb.goal_estimated_arrival_time
                     << ", goal progress: " << rfb.goal_progress << ", distance to finish: " << rfb.distance_to_finish
                     << ", finish estimated arrival time: " << rfb.finish_estimated_arrival_time);

    // Add to the list at index i
    json_msgs[i] = std::move(robot_json);
  }

  // Create the main JSON message
  json json_msg = {{"progress", feedback->info.progress}, {"mission_state", feedback->info.state}, {"message", feedback->info.message}, {"robots", json_msgs}};

  sendTelemetryJsonMessage("MissionFeedback", json_msg);
}

void IROCBridge::missionFeedbackCallback(iroc_fleet_manager::IROCFleetManagerActionFeedback::ConstPtr msg) {
  auto robot_feedbacks = msg->feedback.info.robot_feedbacks;

  // Create a list for robot feedback
  json json_msgs = json::list();

  // Collect each robot feedback and create a json for each
  for (size_t i = 0; i < robot_feedbacks.size(); i++) {
    const auto& rfb = robot_feedbacks[i];

    json robot_json = {{"robot_name", rfb.name},
                       {"message", rfb.message},
                       {"mission_progress", rfb.mission_progress},
                       {"current_goal", rfb.goal_idx},
                       {"distance_to_goal", rfb.distance_to_closest_goal},
                       {"goal_estimated_arrival_time", rfb.goal_estimated_arrival_time},
                       {"goal_progress", rfb.goal_progress},
                       {"distance_to_finish", rfb.distance_to_finish},
                       {"finish_estimated_arrival_time", rfb.finish_estimated_arrival_time}};

    ROS_DEBUG_STREAM("[IROCBridge]: Mission feedback for robot: "
                     << rfb.name << ", message: " << rfb.message << ", progress: " << rfb.mission_progress << ", current goal: " << rfb.goal_idx
                     << ", distance to goal: " << rfb.distance_to_closest_goal << ", goal estimated arrival time: " << rfb.goal_estimated_arrival_time
                     << ", goal progress: " << rfb.goal_progress << ", distance to finish: " << rfb.distance_to_finish
                     << ", finish estimated arrival time: " << rfb.finish_estimated_arrival_time);

    // Add to the list at index i
    json_msgs[i] = std::move(robot_json);
  }

  // Create the main JSON message
  json json_msg = {{"progress", msg->feedback.info.progress}, {"mission_state", msg->feedback.info.state}, {"message", msg->feedback.info.message}, {"robots", json_msgs}};

  sendTelemetryJsonMessage("MissionFeedback", json_msg);
}
//}

// --------------------------------------------------------------
// |                 parsing and output methods                 |
// --------------------------------------------------------------

/* parseGeneralRobotInfo() //{ */
void IROCBridge::parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string& robot_name) {

  json json_msg = {
      {"robot_name", general_robot_info->robot_name},
      {"robot_type", general_robot_info->robot_type},
      {"robot_ip_address", general_robot_info->robot_ip_address},
      {"battery_state",
       {{"voltage", general_robot_info->battery_state.voltage},
        {"percentage", general_robot_info->battery_state.percentage},
        {"wh_drained", general_robot_info->battery_state.wh_drained}}},
      {"ready_to_start", general_robot_info->ready_to_start},
      {"problems_preventing_start", json::list(general_robot_info->problems_preventing_start.begin(), general_robot_info->problems_preventing_start.end())},
      {"errors", json::list(general_robot_info->errors.begin(), general_robot_info->errors.end())}};

  sendTelemetryJsonMessage("GeneralRobotInfo", json_msg);
}
//}

/* parseStateEstimationInfo() //{ */

void IROCBridge::parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string& robot_name) {
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
//}

/* parseControlInfo() //{ */
void IROCBridge::parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name) {
  json json_msg = {{"robot_name", robot_name},
                   {"active_controller", control_info->active_controller},
                   {"available_controllers", json::list(control_info->available_controllers.begin(), control_info->available_controllers.end())},
                   {"active_tracker", control_info->active_tracker},
                   {"available_trackers", json::list(control_info->available_trackers.begin(), control_info->available_trackers.end())},
                   {"thrust", control_info->thrust}};

  sendTelemetryJsonMessage("ControlInfo", json_msg);
}
//}

/* parseCollisionAvoidanceInfo() //{ */
void IROCBridge::parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name) {
  json json_msg = {
      {"robot_name", robot_name},
      {"collision_avoidance_enabled", collision_avoidance_info->collision_avoidance_enabled},
      {"avoiding_collision", collision_avoidance_info->avoiding_collision},
      {"other_robots_visible", json::list(collision_avoidance_info->other_robots_visible.begin(), collision_avoidance_info->other_robots_visible.end())},
  };

  sendTelemetryJsonMessage("CollisionAvoidanceInfo", json_msg);
}
//}

/* parseUavInfo() //{ */
void IROCBridge::parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name) {
  json json_msg = {{"robot_name", robot_name},
                   {"armed", uav_info->armed},
                   {"offboard", uav_info->offboard},
                   {"flight_state", uav_info->flight_state},
                   {"flight_duration", uav_info->flight_duration},
                   {"mass_nominal", uav_info->mass_nominal}};

  sendTelemetryJsonMessage("UavInfo", json_msg);
}
//}

/* parseSensorInfo() //{ */
void IROCBridge::parseSensorInfo(mrs_robot_diagnostics::SensorInfo::ConstPtr sensor_info, const std::string& robot_name) {

  crow::json::rvalue json_msg = crow::json::load(sensor_info->details);
  if (!json_msg) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Could not parse sensor details JSON string: " << sensor_info->details);
    return;
  }
  json telemetry_msg = {
      {"robot_name", robot_name},
      {"sensor_type", sensor_info->type},
      {"details", json_msg}};

  sendTelemetryJsonMessage("SensorInfo", telemetry_msg);
}

/* parseSystemHealthInfo() //{ */
void IROCBridge::parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info, const std::string& robot_name) {
  // Create arrays for node_cpu_loads
  json node_cpu_loads = json::list();
  for (size_t i = 0; i < system_health_info->node_cpu_loads.size(); i++) {
    const auto& node_cpu_load = system_health_info->node_cpu_loads[i];

    // Create a nested array for each node_cpu_load using initializer list
    json node_entry = json::list({node_cpu_load.node_name, node_cpu_load.cpu_load});

    node_cpu_loads[i] = std::move(node_entry);
  }

  // Create array for available_sensors
  json available_sensors = json::list();
  for (size_t i = 0; i < system_health_info->available_sensors.size(); i++) {
    const auto& available_sensor = system_health_info->available_sensors[i];

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
//}

// --------------------------------------------------------------
// |                       helper methods                       |
// --------------------------------------------------------------
/* sendJsonMessage() //{ */
void IROCBridge::sendJsonMessage(const std::string& msg_type, json& json_msg) {
  const std::string url = "/api/mission/" + msg_type;
  const std::string body = json_msg.dump();
  const std::string content_type = "application/json";
  const auto res = http_client_->Post(url, body, content_type);

  if (res)
    ROS_DEBUG_STREAM_THROTTLE(1.0, res->status << ": " << res->body);
  else
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to send PATCH request to address \"" << url << "\": " << to_string(res.error()));

  return;
}
//}

/* sendTelemetryJsonMessage() //{ */
void IROCBridge::sendTelemetryJsonMessage(const std::string& type, json& json_msg) {

  json_msg["type"] = type;
  std::string message = json_msg.dump();

  if (active_telemetry_connection_) {
    try {
      active_telemetry_connection_->send_text(message);
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("Websocket send_text failed, removing connection: " << e.what());
      active_telemetry_connection_ = nullptr;
    }
  }
}
//}

/* callService() //{ */
template <typename Svc_T>
IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc, typename Svc_T::Request req) {
  typename Svc_T::Response res;
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

/* callService() //{ */
template <typename Svc_T>
IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc, typename Svc_T::Request req, typename Svc_T::Response &res) {
  if (sc.call(req, res)) {
    if (res.success) {
      ROS_INFO_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {true, res.message};
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
      return {false, res.message};
    }
  } else {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

template <typename Svc_T>
IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc) {
  return callService<Svc_T>(sc, {});
}

IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc, const bool val) {
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
}
//}

/* routine_death_check() method //{ */
void IROCBridge::routine_death_check() {
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

/* findRobotHandler() method //{ */
IROCBridge::robot_handler_t* IROCBridge::findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers) {
  for (auto& rh : robot_handlers_.handlers) {
    if (rh.robot_name == robot_name)
      return &rh;
  }

  return nullptr;
}
//}

/* getServiceClient() method //{ */
ros::ServiceClient* IROCBridge::getServiceClient(IROCBridge::robot_handler_t* rh_ptr, const IROCBridge::CommandType command_type) {
  switch (command_type) {
    case CommandType::Takeoff:
      return &(rh_ptr->sc_takeoff);
    case CommandType::Land:
      return &(rh_ptr->sc_land);
    case CommandType::Hover:
      return &(rh_ptr->sc_hover);
    case CommandType::Home:
      return &(rh_ptr->sc_land_home);
    case CommandType::Set_Origin:
      return &(rh_ptr->sc_set_origin);
    case CommandType::Set_SafetyBorder:
      return &(rh_ptr->sc_set_safety_area);
    case CommandType::Set_Obstacle:
      return &(rh_ptr->sc_set_obstacle);
    default:
      return nullptr;
  }
}

//}

/* commandAction() method //{ */
IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string>& robot_names, const std::string& command_type) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  crow::status status_code = crow::status::ACCEPTED;
  ss << "Command: " << command_type << " Result: ";

  CommandType command_type_e = CommandType::Unknown;
  auto it = command_type_map_.find(command_type);
  if (it != command_type_map_.end()) {
    command_type_e = it->second;
  } else {
    ss << "Command type \"" << command_type << "\" not found, skipping\n";
    ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Command type \"" << command_type << "\" not found. Skipping.");
    everything_ok = false;
    status_code = crow::status::NOT_FOUND;
  }

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM("Calling command \"" << command_type << "\" .");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      auto* client_ptr = getServiceClient(rh_ptr, command_type_e);
      if (client_ptr != nullptr) {
        const auto resp = callService<std_srvs::Trigger>(*client_ptr);
        if (!resp.success) {
          ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
          everything_ok = false;
          status_code = crow::status::BAD_REQUEST;
        }
      } else {
        ss << "Command type \"" << command_type << "\" not found, skipping\n";
        ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Command type \"" << command_type << "\" not found. Skipping.");
        everything_ok = false;
        status_code = crow::status::NOT_FOUND;
      }

    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
      status_code = crow::status::NOT_FOUND;
    }
  }

  ss << "Successully processed\n";
  return {everything_ok, ss.str(), status_code};
}

template <typename Svc_T>
IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string>& robot_names, const std::string& command_type,
                                                      typename Svc_T::Request req) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  crow::status status_code = crow::status::ACCEPTED;
  ss << "Command: " << command_type << " Result: ";

  CommandType command_type_e = CommandType::Unknown;
  auto it = command_type_map_.find(command_type);
  if (it != command_type_map_.end()) {
    command_type_e = it->second;
  } else {
    ss << "Command type \"" << command_type << "\" not found, skipping\n";
    ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Command type \"" << command_type << "\" not found. Skipping.");
    everything_ok = false;
    status_code = crow::status::NOT_FOUND;
  }

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM("Calling command \"" << command_type << "\" .");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      auto* client_ptr = getServiceClient(rh_ptr, command_type_e);
      if (client_ptr != nullptr) {
        const auto resp = callService<Svc_T>(*client_ptr, req);
        if (!resp.success) {
          ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
          everything_ok = false;
          status_code = crow::status::BAD_REQUEST;
        }
      } else {
        ss << "Command type \"" << command_type << "\" not found, skipping\n";
        ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Command type \"" << command_type << "\" not found. Skipping.");
        everything_ok = false;
        status_code = crow::status::NOT_FOUND;
      }

    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_WARN_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
      status_code = crow::status::NOT_FOUND;
    }
  }
  ss << "Successully processed\n";

  return {everything_ok, ss.str(), status_code};
}
//}

/* resultToJson() method //{ */

template <typename Result>
json resultToJson(const boost::shared_ptr<const Result>& result) {
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
//}

/* missionGoalToJson() method //{ */

json missionGoalToJson(const iroc_fleet_manager::IROCFleetMissionGoal &mission_goal) {
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
      mrs_msgs::Reference reference = points.at(j).reference;
      json point                    = {{"x", reference.position.x}, {"y", reference.position.y}, {"z", reference.position.z}, {"heading", reference.heading}};
      points_list[j]                = std::move(point);
    }
    json mission    = {{"points", points_list}, {"frame_id", frame_id}, {"height_id", height_id}};
    json robot_data = {{"robot", robot_name}, {"success", true}, {"message", "Mission loaded successfully"}, {"mission", mission}};
    robots_data[i]  = std::move(robot_data);
  }

  json response = {{"success", true}, {"message", "Mission uploaded successfully"}, {"type", mission_goal.type}, {"uuid", mission_goal.uuid}, {"robot_data", robots_data}};

  return response;
}
//}


// --------------------------------------------------------------
// |                     REST API callbacks                     |
// --------------------------------------------------------------

/* pathCallback() method //{ */

/**
 * \brief Callback for the path endpoint. It parses the JSON message and publishes it to the corresponding robot handler.
 *
 * \param req The HTTP request containing the JSON message.
 * \return A response indicating the success or failure of the operation.
 */
crow::response IROCBridge::pathCallback(const crow::request& req) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a path message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  std::string robot_name = json_msg["robot_name"].s();
  std::scoped_lock lck(robot_handlers_.mtx);
  auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
  if (!rh_ptr) {
    ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Robot \"" + robot_name + "\" not found, ignoring\"}");
  }

  std::string frame_id = json_msg["frame_id"].s();
  if (frame_id.empty()) {
    ROS_WARN_STREAM("[IROCBridge]: Bad frame_id input: Expected a string.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad frame_id input: Expected a string.\"}");
  }

  std::vector<crow::json::rvalue> points = json_msg["points"].lo();
  if (points.empty()) {
    ROS_WARN_STREAM("[IROCBridge]: Bad points input: Expected an array.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad points input: Expected an array.\"}");
  }

  mrs_msgs::Path msg_path;
  msg_path.points.reserve(points.size());
  msg_path.header.stamp = ros::Time::now();
  msg_path.header.frame_id = frame_id;
  msg_path.fly_now = true;
  msg_path.use_heading = false;

  for (const auto& el : points) {
    mrs_msgs::Reference ref;
    ref.position.x = el["x"].d();
    ref.position.y = el["y"].d();
    ref.position.z = el["z"].d();

    if (el.has("heading")) {
      ref.heading = el["heading"].d();
      msg_path.use_heading = true;
    }

    msg_path.points.push_back(ref);
  }

  // Publish the path
  rh_ptr->pub_path.publish(msg_path);

  ROS_INFO_STREAM("[IROCBridge]: Set a path with " << points.size() << " length.");
  return crow::response(crow::status::OK, "{\"message\": \"Set a path with " + std::to_string(points.size()) + " length.\"}");
}
//}

/* setOriginCallback() method //{ */

/**
 * \brief Callback for the set safety border request. It receives a list of points and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setOriginCallback(const crow::request& req) {

  ROS_INFO_STREAM("[IROCBridge]: Parsing a setOriginCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  // Get message properties
  int frame_id = json_msg["frame_id"].i();

  // The service supports latlon and UTM, but we we can at the moment support only latlon for IROC
  // TODO: It can be extended in the future to support UTM origin
  mrs_msgs::ReferenceStampedSrv::Request service_request;
  service_request.header.frame_id = "latlon_origin";
  service_request.header.stamp = ros::Time::now();
  service_request.reference.position.x = json_msg["x"].d();
  service_request.reference.position.y = json_msg["y"].d();

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers

  const auto result = commandAction<mrs_msgs::ReferenceStampedSrv>(robot_names, "set_origin", service_request);

  if (result.success) {
    ROS_INFO_STREAM("[IROCBridge]: Set origin for " << robot_names.size() << " robots.");
    world_origin_.x = json_msg["x"].d();
    world_origin_.y = json_msg["y"].d();
  }

  return crow::response(result.status_code, result.message);
}
//}

/* getOriginCallback() method //{ */

/**
 * \brief Callback to get the world origin. 
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getOriginCallback(const crow::request &req) {

  ROS_INFO_STREAM("[IROCBridge]: Processing a getOriginCallback");

  iroc_fleet_manager::GetWorldOriginSrv world_origin_service;

  const auto resp = callService<iroc_fleet_manager::GetWorldOriginSrv>(sc_get_world_origin_, world_origin_service.request, world_origin_service.response);

  json json_msg;
  if (!resp.success) {
    json_msg["message"] = "Call was not successful with message: " + resp.message;
    ROS_WARN_STREAM("[IROCBridge]: " << json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {
    json_msg["message"] = world_origin_service.response.message;
    json_msg["x"]       = world_origin_service.response.origin_x;
    json_msg["y"]       = world_origin_service.response.origin_y;
    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }
}
//}


/* setSafetyBorderCallback() method //{ */

/**
 * \brief Callback for the set safety border request. It receives a list of points and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setSafetyBorderCallback(const crow::request& req) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a setSafetyBorderCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  bool enabled = true; // Defined default as true

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
    ROS_WARN_STREAM("[IROCBridge]: Unknown height_id: " << height_id);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Unknown height_id field\"}");
  }

  std::vector<mrs_msgs::Point2D> border_points;
  border_points.reserve(points.size());

  for (const auto& el : points) {
    mrs_msgs::Point2D pt;
    pt.x = el["x"].d();
    pt.y = el["y"].d();

    border_points.push_back(pt);
  }

  ROS_INFO("[IROCBridge]: Border points size %zu ", border_points.size());

  mrs_msgs::SafetyBorder safety_border;
  safety_border.enabled = enabled;
  safety_border.horizontal_frame = horizontal_frame;
  safety_border.vertical_frame = vertical_frame;
  safety_border.points = border_points;
  safety_border.max_z = max_z;
  safety_border.min_z = min_z;

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers
  mrs_msgs::SetSafetyBorderSrvRequest service_request;
  service_request.safety_border  = safety_border;
  service_request.keep_obstacles = false;
  mrs_msgs::SetSafetyBorderSrv::Request req_srv = service_request;

  const auto result = commandAction<mrs_msgs::SetSafetyBorderSrv>(robot_names, "set_safety_border", req_srv);

  return crow::response(result.status_code, result.message);
}
//}

/* getSafetyBorderCallback() method //{ */

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

  return id;
}

crow::response IROCBridge::getSafetyBorderCallback(const crow::request &req) {

  ROS_INFO_STREAM("[IROCBridge]: Processing a getSafetyBorderCallback message ROS -> JSON.");
  iroc_fleet_manager::GetSafetyBorderSrv get_safety_border_service;

  const auto resp = callService<iroc_fleet_manager::GetSafetyBorderSrv>(sc_get_safety_border_, get_safety_border_service.request, get_safety_border_service.response);

  json json_msg;
  if (!resp.success) {
    json_msg["message"] = "Call was not successful with message: " + resp.message;
    ROS_WARN_STREAM("[IROCBridge]: " << json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {
    json_msg["message"] = get_safety_border_service.response.message;

    json points                   = json::list();
    auto vector_points            = get_safety_border_service.response.border.points; 
    double max_z                  = get_safety_border_service.response.border.max_z; 
    double min_z                  = get_safety_border_service.response.border.min_z; 
    std::string  horizontal_frame = get_safety_border_service.response.border.horizontal_frame; 
    std::string  vertical_frame   = get_safety_border_service.response.border.vertical_frame; 

    for (size_t i = 0; i < vector_points.size(); i++) {
      const auto point = vector_points.at(i);
      json point_json  = {{"x", point.x}, {"y", point.y}};
      points[i] = std::move(point_json);
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
//}

/* setObstacleCallback() method //{ */

/**
 * \brief Callback for the set obstacle request. It receives a list of obstacles for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setObstacleCallback(const crow::request &req) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a setObstacleCallback message JSON -> ROS.");
  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg)
    return crow::response(crow::status::BAD_REQUEST, "Failed to parse JSON" + req.body);

  // Check if obstacles array exists
  if (!json_msg.has("obstacles") || !json_msg["obstacles"]) {
    return crow::response(crow::status::BAD_REQUEST, "Missing obstacles array: " + req.body);
  }

  std::vector<crow::json::rvalue> obstacles = json_msg["obstacles"].lo();
  if (obstacles.empty()) {
    return crow::response(crow::status::BAD_REQUEST, "Empty obstacles array: " + req.body);
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
      return crow::response(crow::status::BAD_REQUEST, "Missing required fields in obstacle " + std::to_string(i) + ": " + req.body);
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
      return crow::response(crow::status::BAD_REQUEST, "Unknown height_id field in obstacle " + std::to_string(i) + ": " + req.body);
    }

    if (points.empty()) {
      return crow::response(crow::status::BAD_REQUEST, "Empty points array in obstacle " + std::to_string(i) + ": " + req.body);
    }

    // Process points
    std::vector<mrs_msgs::Point2D> border_points;
    border_points.reserve(points.size());
    for (const auto &el : points) {
      if (!el.has("x") || !el.has("y")) {
        return crow::response(crow::status::BAD_REQUEST, "Missing x or y in point for obstacle " + std::to_string(i) + ": " + req.body);
      }
      mrs_msgs::Point2D pt;
      pt.x = el["x"].d();
      pt.y = el["y"].d();
      border_points.push_back(pt);
    }

    // Logging
    ROS_INFO("[IROCBridge]: Obstacle %zu border points size %zu", i, border_points.size());

    // Create obstacle request
    mrs_msgs::SetObstacleSrvRequest obstacle_req;
    obstacle_req.horizontal_frame = horizontal_frame;
    obstacle_req.vertical_frame   = vertical_frame;
    obstacle_req.points           = border_points;
    obstacle_req.max_z            = max_z;
    obstacle_req.min_z            = min_z;

    // Call service for this obstacle
    const auto result = commandAction<mrs_msgs::SetObstacleSrv>(robot_names, "set_obstacle", obstacle_req);

    // Check if the service call failed
    if (!result.success) {
      ROS_ERROR("[IROCBridge]: Failed to set obstacle %zu: %s", i, result.message.c_str());
      return crow::response(result.status_code, "Failed to set obstacle " + std::to_string(i) + ": " + result.message);
    }
  }

  ROS_INFO("[IROCBridge]: Successfully set %zu obstacles", obstacles.size());
  return crow::response(crow::status::OK, "Successfully set " + std::to_string(obstacles.size()) + " obstacles");
}
//}

/* getObstaclesCallback() method //{ */

/**
 * \brief Callback to get the obstacles. 
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getObstaclesCallback(const crow::request &req) {

  ROS_INFO_STREAM("[IROCBridge]: Processing a getObstaclesCallback message ROS -> JSON.");
  iroc_fleet_manager::GetObstaclesSrv get_obstacles_service;

  const auto resp = callService<iroc_fleet_manager::GetObstaclesSrv>(sc_get_obstacles_, get_obstacles_service.request, get_obstacles_service.response);

  json json_msg;
  if (!resp.success) {
    json_msg["message"] = "Call was not successful with message: " + resp.message;
    ROS_WARN_STREAM("[IROCBridge]: " << json_msg["message"].dump());
    return crow::response(crow::status::CONFLICT, json_msg.dump());
  } else {

    json_msg["message"]      = get_obstacles_service.response.message;
    int number_of_obstacles  = get_obstacles_service.response.obstacles.rows.size();
    auto obstacles           = get_obstacles_service.response.obstacles;
    json obstacles_json_list = json::list(); 

    size_t point_index = 0; 

    for (size_t i = 0; i < number_of_obstacles; i++) {
      int number_of_vertices = obstacles.rows.at(i);
      double max_z           = obstacles.max_z.at(i); 
      double min_z           = obstacles.min_z.at(i); 

      // Extract the points for the obstacle
      json points_list = json::list();
      for (int j = 0; j < number_of_vertices; j++) {
        if (point_index < obstacles.data.size()) {
          json point = {{"x", obstacles.data.at(point_index).x}, {"y", obstacles.data.at(point_index).y}};
          points_list[j] = std::move(point);
          point_index++; // Move to next point
        }
      }

      json obstacle_json     = {{"points", points_list},
                                {"max_z", max_z},
                                {"min_z", min_z},
                                {"frame_id", getFrameID(obstacles.horizontal_frame)},
                                {"height_id", getFrameID(obstacles.vertical_frame)}};
      obstacles_json_list[i] = std::move(obstacle_json);
    }

    json json_msg = {{"message", "All robots in the fleet with the same obstacles"}, {"obstacles", obstacles_json_list}};
    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }
}
//}

/* getMissionCallback() method //{ */

/**
 * \brief Callback to get the loaded mission. 
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::getMissionCallback(const crow::request &req) {

  ROS_INFO_STREAM("[IROCBridge]: Processing a getMissionCallback message ROS -> JSON.");
  iroc_fleet_manager::GetMissionPointsSrv get_mission_data_service;
  const auto resp =
      callService<iroc_fleet_manager::GetMissionPointsSrv>(sc_get_mission_data_, get_mission_data_service.request, get_mission_data_service.response);

  if (!resp.success) {
    json error_response;
    error_response["message"]    = resp.message;
    error_response["success"]    = false;
    error_response["robot_data"] = json::list();
    ROS_WARN_STREAM("[IROCBridge]: " << error_response["message"].dump());
    return crow::response(crow::status::INTERNAL_SERVER_ERROR, error_response.dump());
  }

  auto mission_goal = get_mission_data_service.response.mission_goal;
  auto json         = missionGoalToJson(mission_goal);
  return crow::response(crow::status::OK, json.dump());
}
//}

/* missionCallback() method //{ */

/**
 * \brief Callback for the waypoint mission request. It receives a list of missions for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::missionCallback(const crow::request &req) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a missionCallback message JSON -> ROS.");

  try {
    crow::json::rvalue json_msg = crow::json::load(req.body);
    if (!json_msg || !json_msg.has("type") || !json_msg.has("details"))
      return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad request: Failed to parse JSON or missing 'type' and 'details' keys\"}");

    // Validate if the action client is connected and if the action is already running
    if (!fleet_manager_action_client_->isServerConnected()) {
      ROS_WARN_STREAM("[IROCBridge]: Action server is not connected. Check the iroc_fleet_manager node.");
      std::string msg = "Action server is not connected. Check iroc_fleet_manager node.\n";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    } else if (!fleet_manager_action_client_->getState().isDone()) {
      ROS_WARN_STREAM("[IROCBridge]: Mission is already running. Terminate the previous one, or wait until it is finished.");
      std::string msg = "Mission is already running. Terminate the previous one, or wait until it is finished";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }

    // Send the action goal to the fleet manager
    FleetManagerGoal action_goal;
    std::string type = json_msg["type"].s();
    std::string uuid = json_msg["uuid"].s();

    // Convert rvalue to wvalue, then dump to string
    crow::json::wvalue details_wvalue(json_msg["details"]);
    std::string details = details_wvalue.dump();

    action_goal.type    = type;
    action_goal.uuid    = uuid;
    action_goal.details = details;

    fleet_manager_action_client_->sendGoal(
        action_goal, [this](const auto &state, const auto &result) { missionDoneCallback<iroc_fleet_manager::IROCFleetManagerResult>(state, result); },
        [this]() { missionActiveCallback(); },
        [this](const auto &feedback) { missionFeedbackCallback<iroc_fleet_manager::IROCFleetManagerFeedback>(feedback); });

    // Waiting in the case the trajectories are rejected. We can better wait will the state is pending
    ros::Duration(5).sleep();

    if (fleet_manager_action_client_->getState().isDone()) { // If the action is done, the action finished instantly
      auto result        = fleet_manager_action_client_->getResult();
      auto json          = resultToJson(result);
      const auto message = result->message;
      ROS_WARN("[IROCBridge]: %s", message.c_str());
      return crow::response(crow::status::BAD_REQUEST, json);
    }

    // TODO to replace with proper action response in ROS2
    iroc_fleet_manager::GetMissionPointsSrv get_mission_data_service;
    const auto resp =
        callService<iroc_fleet_manager::GetMissionPointsSrv>(sc_get_mission_data_, get_mission_data_service.request, get_mission_data_service.response);

    if (!resp.success) {
      json error_response;
      error_response["message"]     = resp.message;
      error_response["success"]     = false;
      error_response["robot_data"]  = json::list();
      ROS_WARN_STREAM("[IROCBridge]: " << error_response["message"].dump());
      return crow::response(crow::status::INTERNAL_SERVER_ERROR, error_response.dump());
    }

    auto mission_goal = get_mission_data_service.response.mission_goal;
    auto json = missionGoalToJson(mission_goal);
    return crow::response(crow::status::CREATED, json.dump());
  }
  catch (const std::exception &e) {
    ROS_WARN_STREAM("[IROCBridge]: Failed to parse JSON from message: " << e.what());
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Failed to parse JSON from message: " + std::string(e.what()) + "\"}");
  }
}
//}

/* changeFleetMissionStateCallback() method //{ */

/**
 * \brief Callback that changes the mission state of the fleet, either starting, stopping, or pausing it.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::changeFleetMissionStateCallback(const crow::request& req, const std::string& type) {
  std::scoped_lock lck(robot_handlers_.mtx);

  // Input validation
  if (type != "start" && type != "stop" && type != "pause")
    return crow::response(crow::status::NOT_FOUND);

  mrs_msgs::String ros_srv;
  ros_srv.request.value = type;

  const auto resp = callService<mrs_msgs::String>(sc_change_fleet_mission_state_, ros_srv.request);
  if (!resp.success)
    return crow::response(crow::status::INTERNAL_SERVER_ERROR, "{\"message\": \"" + resp.message + "\"}");
  else
    return crow::response(crow::status::ACCEPTED, "{\"message\": \"" + resp.message + "\"}");
}
//}

/* changeRobotMissionStateCallback() method //{ */

/**
 * \brief Callback that changes the mission state of a specific robot, either starting, stopping, or pausing it.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::changeRobotMissionStateCallback(const crow::request& req, const std::string& robot_name, const std::string& type) {
  std::scoped_lock lck(robot_handlers_.mtx);

  // Input validation
  if (type != "start" && type != "stop" && type != "pause")
    return crow::response(crow::status::NOT_FOUND);
  if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(), [&robot_name](const auto& rh) { return rh.robot_name == robot_name; }))
    return crow::response(crow::status::NOT_FOUND, "Robot not found");

  json json_msg;

  iroc_fleet_manager::ChangeRobotMissionStateSrv ros_srv;
  ros_srv.request.robot_name = robot_name;
  ros_srv.request.type = type;

  const auto resp = callService<iroc_fleet_manager::ChangeRobotMissionStateSrv>(sc_change_robot_mission_state_, ros_srv.request);
  if (!resp.success) {
    json_msg["message"] = "Call was not successful with message: " + resp.message;
    ROS_WARN_STREAM("[IROCBridge]: " << json_msg["message"].dump());

    return crow::response(crow::status::INTERNAL_SERVER_ERROR, json_msg.dump());
  } else {
    json_msg["message"] = "Call successful";
    return crow::response(crow::status::ACCEPTED, json_msg.dump());
  }

  return crow::response(crow::status::NOT_FOUND);
}
//}

/* commandCallback() method //{ */

/**
 * \brief Callback that sends a `hover` command to the robots specified in the request body.
 * \param req Crow request
 *
 * \return res Crow response
 */
crow::response IROCBridge::commandCallback(const crow::request& req, const std::string& command_type, std::optional<std::string> robot_name) {
  std::scoped_lock lck(robot_handlers_.mtx);
  std::vector<std::string> robot_names;

  if (robot_name.has_value()) {
    robot_names.push_back(robot_name.value());
  } else {
    robot_names.reserve(robot_handlers_.handlers.size());
    for (const auto& rh : robot_handlers_.handlers) {
      robot_names.push_back(rh.robot_name);
    }
  }

  const auto result = commandAction(robot_names, command_type);
  return crow::response(result.status_code, result.message);
}
//}

/* availableRobotsCallback() method //{ */

/**
 * \brief Callback that returns a list of available robot names in the fleet based on `robot_handlers_` vector.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::availableRobotsCallback([[maybe_unused]] const crow::request& req) {
  if (robot_handlers_.handlers.empty()) {
    ROS_WARN_STREAM("[IROCBridge]: No robots available in the fleet.");
    return crow::response(crow::status::NO_CONTENT, "{\"message\": \"No robots available in the fleet.\"}");
  }

  json robots = json::list();
  for (size_t i = 0; i < robot_handlers_.handlers.size(); i++) {
    if (!robot_handlers_.handlers[i].sh_general_robot_info.hasMsg()) {
      ROS_WARN_STREAM("[IROCBridge]: Robot handler for robot " << robot_handlers_.handlers[i].robot_name << " does not have general robot info.");
      continue;
    }

    robots[robots.size()] = {
      {"name", robot_handlers_.handlers[i].sh_general_robot_info.getMsg()->robot_name},
      {"type", robot_handlers_.handlers[i].sh_general_robot_info.getMsg()->robot_type}
    };
  }

  return crow::response(crow::status::ACCEPTED, robots.dump());
}
//}

/* remoteControlCallback() method //{ */

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
void IROCBridge::remoteControlCallback(crow::websocket::connection& conn, const std::string& data, bool is_binary) {
  // Convert and check if the received data is a valid JSON
  crow::json::rvalue json_data = crow::json::load(data);
  if (!json_data || !json_data.has("command") || !json_data.has("data")) {
    ROS_WARN_STREAM("[IROCBridge]: Failed to parse JSON from websocket message: " << data);
    conn.send_text("{\"error\": \"Failed to parse JSON or missing 'command'/'data'\"}");
    return;
  }

  json json_response;
  std::string command = json_data["command"].s();
  if (command == "message") {
    ROS_INFO_STREAM("[IROCBridge]: Received message from " << conn.get_remote_ip() << ": " << json_data["data"].s());
    conn.send_text("{\"status\": \"Ok, received message\"}");
  } else if (command == "move") {
    std::scoped_lock lck(robot_handlers_.mtx);

    // Robot id validation
    if (!json_data.has("robot_name")) {
      ROS_WARN_STREAM("[IROCBridge]: Missing robot_id in websocket message: " << data);
      conn.send_text("{\"error\": \"Missing robot_id\"}");
      return;
    }

    std::string robot_name = json_data["robot_name"].s();
    if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(),
                     [robot_name](const robot_handler_t& rh) { return rh.robot_name == robot_name; })) {
      ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
      conn.send_text("{\"error\": \"Robot not found\"}");
      return;
    }

    crow::json::rvalue movement_data = json_data["data"];

    mrs_msgs::VelocityReferenceStampedSrvRequest req;
    req.reference.header.frame_id = "fcu_untilted";

    req.reference.reference.velocity.x = movement_data["x"].d() * max_linear_speed_;
    req.reference.reference.velocity.y = movement_data["y"].d() * max_linear_speed_;
    req.reference.reference.velocity.z = movement_data["z"].d() * max_linear_speed_;
    req.reference.reference.heading_rate = movement_data["heading"].d() * max_heading_rate_;
    req.reference.reference.use_heading_rate = true;

    auto* robot_handler_ptr = findRobotHandler(robot_name, robot_handlers_);
    auto res = callService<mrs_msgs::VelocityReferenceStampedSrv>(robot_handler_ptr->sc_velocity_reference, req);

    if (res.success) {
      json_response["ok"] = true;
      json_response["message"] = "Movement command sent";
    } else {
      json_response["ok"] = false;
      json_response["message"] = "Failed to send movement command: " + res.message;
    }
  } else {
    ROS_WARN_STREAM("[IROCBridge]: Unknown command in websocket message: " << command);
    json_response["ok"] = false;
    json_response["message"] = "Unknown command";
  }
  conn.send_text(json_response.dump());
}
//}

} // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
