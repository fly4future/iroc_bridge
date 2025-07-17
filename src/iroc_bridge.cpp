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
#include <mrs_robot_diagnostics/enums/robot_type.h>

#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/CoverageMissionAction.h>
#include <iroc_fleet_manager/CoverageMissionRobot.h>
#include <iroc_fleet_manager/AutonomyTestAction.h>

#include <iroc_mission_handler/MissionAction.h>

#include <unistd.h>
#include <iostream>
//}

namespace iroc_bridge
{

using json = crow::json::wvalue; 

using vec3_t = Eigen::Vector3d;
using vec4_t = Eigen::Vector4d;

using namespace actionlib;

typedef SimpleActionClient<iroc_fleet_manager::WaypointFleetManagerAction> WaypointFleetManagerClient;
typedef SimpleActionClient<iroc_fleet_manager::CoverageMissionAction> CoveragePlannerClient;
typedef SimpleActionClient<iroc_fleet_manager::AutonomyTestAction> AutonomyTestClient;
//Waypoint mission goal
typedef iroc_fleet_manager::WaypointFleetManagerGoal FleetManagerActionServerGoal;
// Autonomy test goal
typedef iroc_fleet_manager::AutonomyTestGoal AutonomyTestActionServerGoal;
// Coverage mission goal 
typedef iroc_fleet_manager::CoverageMissionGoal coverageMissionActionServerGoal;

typedef mrs_robot_diagnostics::robot_type_t robot_type_t;

/* class IROCBridge //{ */
class IROCBridge : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  // | ---------------------- HTTP REST API --------------------- |
  std::thread                  th_http_srv_;
  crow::App<crow::CORSHandler> http_srv_;

  std::unique_ptr<httplib::Client> http_client_;


  struct result_t
  {
    bool        success;
    std::string message;
  };

  struct action_result_t
  {
    bool           success;
    std::string    message;
    crow::status   status_code;
  };

  // | ---------------------- Command types --------------------- |
  enum class CommandType {
    Takeoff,
    Land,
    Hover,
    Home,
    Set_Origin,
    Set_SafetyBorder,
    Set_Obstacle,
    Unknown
  };

  enum class Change_SvC_T {
      FleetWaypoint,
      RobotWaypoint,
      FleetCoverage,
      RobotCoverage,
      RobotAutonomyTest
  };

  std::map<std::string, CommandType> command_type_map_ = {
      {"takeoff", CommandType::Takeoff},
      {"land", CommandType::Land},
      {"hover", CommandType::Hover},
      {"home", CommandType::Home},
      {"set_origin", CommandType::Set_Origin},
      {"set_safety_border", CommandType::Set_SafetyBorder},
      {"set_obstacle", CommandType::Set_Obstacle},
  };

  std::map<std::string, Change_SvC_T> change_type_map_ = {
      {"waypoint", Change_SvC_T::FleetWaypoint},
      {"coverage", Change_SvC_T::FleetCoverage},
  };

  std::map<std::string, Change_SvC_T> change_robot_type_map_ = {
      {"waypoint", Change_SvC_T::RobotWaypoint},
      {"coverage", Change_SvC_T::RobotCoverage},
      {"autonomy_test", Change_SvC_T::RobotAutonomyTest},
  };
    

  // | ---------------------- ROS parameters ------------------ |
  double max_linear_speed_;
  double max_heading_rate_;

  // | ---------------------- ROS subscribers --------------------- |

  struct robot_handler_t
  {
    std::string                                                              robot_name;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo>       sh_general_robot_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo>    sh_state_estimation_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo>            sh_control_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo> sh_collision_avoidance_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo>                sh_uav_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo>       sh_system_health_info;

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

  struct robot_handlers_t
  {
    std::recursive_mutex         mtx;
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ----------------------- ROS Clients ----------------------- |
  ros::ServiceClient sc_change_fleet_mission_state;
  ros::ServiceClient sc_change_robot_mission_state;
  ros::ServiceClient sc_change_coverage_mission_state;
  ros::ServiceClient sc_change_coverage_mission_robot_state;
  ros::ServiceClient sc_change_autonomy_test_state;
  std::string        latest_mission_type_;

  // | ----------------- action client callbacks ---------------- |

  // Mission callbacks
  void missionActiveCallback();
  template <typename Result>
  void missionDoneCallback(const SimpleClientGoalState& state, const boost::shared_ptr<const Result>& result);
  template <typename Feedback> 
  void missionFeedbackCallback(const boost::shared_ptr<const Feedback>& feedback); 

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string& robot_name);
  void parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string& robot_name);
  void parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name);
  void parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name);
  void parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name);
  void parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr uav_info, const std::string& robot_name);

  void                sendJsonMessage(const std::string& msg_type,json& json_msg);
  void                sendTelemetryJsonMessage(const std::string& type, json& json_msg);
  robot_handler_t*    findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers);

  ros::ServiceClient* getServiceClient(IROCBridge::robot_handler_t* rh_ptr, const IROCBridge::CommandType command_type);
  ros::ServiceClient  getServiceClient(const IROCBridge::Change_SvC_T service_type);
 

  action_result_t commandAction(const std::vector<std::string>& robot_names, const std::string& command_type);

  template <typename Svc_T> 
  action_result_t commandAction(const std::vector<std::string>& robot_names, const std::string& command_type, typename Svc_T::Request req);

  // REST API callbacks
  crow::response pathCallback(const crow::request& req);
  crow::response setOriginCallback(const crow::request& req);
  crow::response setSafetyBorderCallback(const crow::request& req);
  crow::response setObstacleCallback(const crow::request& req);
  crow::response waypointMissionCallback(const crow::request& req);
  crow::response coverageMissionCallback(const crow::request& req);
  crow::response autonomyTestCallback(const crow::request& req);

  crow::response changeFleetMissionStateCallback(const crow::request& req, const std::string& type);
  crow::response changeRobotMissionStateCallback(const crow::request& req, const std::string& robot_name, const std::string& type);

  crow::response availableRobotsCallback(const crow::request& req);
  crow::response commandCallback(const crow::request& req, const std::string & command_type , std::optional<std::string> robot_name);

  // Websocket callbacks
  void remoteControlCallback(crow::websocket::connection& conn, const std::string& data, bool is_binary);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

  std::thread th_death_check_;
  std::thread th_telemetry_check_;
  void        routine_death_check();
  crow::websocket::connection*           active_telemetry_connection_ = nullptr;
  std::mutex mtx_telemetry_connections_;
  
  std::unique_ptr<WaypointFleetManagerClient> action_client_ptr_;
  std::unique_ptr<CoveragePlannerClient>      coverage_action_client_ptr_;
  std::unique_ptr<AutonomyTestClient>         autonomy_test_client_ptr_;

  // Latlon origin
  mrs_msgs::Point2D  world_origin_;
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
  std::string network_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  param_loader.loadParam("network_config", network_config_path);

  if (network_config_path != "") {
    param_loader.addYamlFile(network_config_path);
  }

  const auto main_timer_rate       = param_loader.loadParam2<double>("main_timer_rate");
  const auto _http_server_threads_ = param_loader.loadParam2<double>("http_server_threads");
  const auto no_message_timeout    = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  const auto url         = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");

  const auto robot_names = param_loader.loadParam2<std::vector<std::string>>("network/robot_names");

  max_linear_speed_  = param_loader.loadParam2<double>("remote_control_limits/max_linear_speed");
  max_heading_rate_ = param_loader.loadParam2<double>("remote_control_limits/max_heading_rate");

  // Remove ground-station hostname from robot names
  std::string              hostname_str(hostname.data());
  std::vector<std::string> filtered_robot_names = robot_names;

  auto it = std::remove(filtered_robot_names.begin(), filtered_robot_names.end(), hostname_str);
  filtered_robot_names.erase(it, filtered_robot_names.end());

  std::cout << "Filtered robot names: ";
  for (const auto& name : filtered_robot_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------- HTTP REST API callbacks ---------------- |
  // Client
  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  // Server
  // Do we need this (set_path)?
  CROW_ROUTE(http_srv_, "/set_path").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return pathCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/origin").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return setOriginCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/borders").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return setSafetyBorderCallback(req); });
  CROW_ROUTE(http_srv_, "/safety-area/obstacles").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return setObstacleCallback(req); });
  CROW_ROUTE(http_srv_, "/mission/waypoints").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return waypointMissionCallback(req); });
  CROW_ROUTE(http_srv_, "/mission/coverage").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return coverageMissionCallback(req); });
  CROW_ROUTE(http_srv_, "/mission/autonomy-test").methods(crow::HTTPMethod::Post)([this](const crow::request& req){ return autonomyTestCallback(req); });

  // Missions
  //TODO: CROW_REGEX_ROUTE(http_srv_, R"(/fleet/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/mission/<string>")
      .methods(crow::HTTPMethod::Post)([this](const crow::request& req, const std::string& type){ return changeFleetMissionStateCallback(req, type); });
  //TODO: CROW_REGEX_ROUTE(http_srv_, R"(/robots/(\w+)/mission/(start|stop|pause))")
  CROW_ROUTE(http_srv_, "/robots/<string>/mission/<string>")
      .methods(crow::HTTPMethod::Post)(
          [this](const crow::request& req, const std::string& robot_name, const std::string& type){ return changeRobotMissionStateCallback(req, robot_name, type); });

  // Available robots endpoint
  CROW_ROUTE(http_srv_, "/robots").methods(crow::HTTPMethod::Get)([this](const crow::request& req){ return availableRobotsCallback(req); });

  // Command endpoints with robot name in the path (land, takeoff, hover, home) 
  CROW_ROUTE(http_srv_, "/robots/<string>/<string>").methods(crow::HTTPMethod::Post)
    ([this](const crow::request& req, const std::string& robot_name, const std::string& command_type) {
     return commandCallback(req, command_type, robot_name);
     });

  // Command endpoint for all robots (land, takeoff, hover, home) 
  CROW_ROUTE(http_srv_, "/robots/<string>").methods(crow::HTTPMethod::Post)
    ([this](const crow::request& req, const std::string& command_type) {
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
  shopts.nh                 = nh_;
  shopts.node_name          = "IROCBridge";
  shopts.no_message_timeout = no_message_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // populate the robot handlers vector
  {
    std::scoped_lock lck(robot_handlers_.mtx);

    robot_handlers_.handlers.reserve(filtered_robot_names.size());
    for (const auto& robot_name : filtered_robot_names) {
      robot_handler_t robot_handler;
      robot_handler.robot_name = robot_name;

      const std::string general_robot_info_topic_name = "/" + robot_name + nh_.resolveName("in/general_robot_info");
      robot_handler.sh_general_robot_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo>(shopts, general_robot_info_topic_name);

      const std::string state_estimation_info_topic_name = "/" + robot_name + nh_.resolveName("in/state_estimation_info");
      robot_handler.sh_state_estimation_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo>(shopts, state_estimation_info_topic_name);

      const std::string control_info_topic_name = "/" + robot_name + nh_.resolveName("in/control_info");
      robot_handler.sh_control_info             = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo>(shopts, control_info_topic_name);

      const std::string collision_avoidance_info_topic_name = "/" + robot_name + nh_.resolveName("in/collision_avoidance_info");
      robot_handler.sh_collision_avoidance_info =
          mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo>(shopts, collision_avoidance_info_topic_name);

      const std::string uav_info_topic_name = "/" + robot_name + nh_.resolveName("in/uav_info");
      robot_handler.sh_uav_info             = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo>(shopts, uav_info_topic_name);

      const std::string system_health_info_topic_name = "/" + robot_name + nh_.resolveName("in/system_health_info");
      robot_handler.sh_system_health_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo>(shopts, system_health_info_topic_name);

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

      robot_handler.sc_velocity_reference = nh_.serviceClient<mrs_msgs::VelocityReferenceStampedSrv>("/" + robot_name + nh_.resolveName("svc/velocity_reference"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/velocity_reference\' -> \'%s\'", robot_handler.sc_velocity_reference.getService().c_str());

      // | ----------------------- publishers ----------------------- |
      robot_handler.pub_path = nh_.advertise<mrs_msgs::Path>("/" + robot_name + nh_.resolveName("out/path"), 2);
      ROS_INFO("[IROCBridge]: Created publisher on topic \'out/path\' -> \'%s\'", robot_handler.pub_path.getTopic().c_str());

      // move is necessary because copy construction of the subscribe handlers is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }

  sc_change_fleet_mission_state = nh_.serviceClient<mrs_msgs::String>(nh_.resolveName("svc/change_fleet_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_fleet_mission_state\' -> \'%s\'",
           sc_change_fleet_mission_state.getService().c_str());

  sc_change_robot_mission_state = nh_.serviceClient<iroc_fleet_manager::ChangeRobotMissionStateSrv>(nh_.resolveName("svc/change_robot_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_robot_mission_state\' -> \'%s\'",
           sc_change_robot_mission_state.getService().c_str());

  sc_change_coverage_mission_state = nh_.serviceClient<mrs_msgs::String>(nh_.resolveName("svc/change_coverage_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_coverage_mission_state\' -> \'%s\'",
           sc_change_coverage_mission_state.getService().c_str());

  sc_change_coverage_mission_robot_state = nh_.serviceClient<iroc_fleet_manager::ChangeRobotMissionStateSrv>(nh_.resolveName("svc/change_coverage_mission_robot_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_robot_mission_state\' -> \'%s\'",
           sc_change_coverage_mission_state.getService().c_str());

  sc_change_autonomy_test_state = nh_.serviceClient<iroc_fleet_manager::ChangeRobotMissionStateSrv>(nh_.resolveName("svc/change_autonomy_test_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_autonomy_test_state\' -> \'%s\'",
           sc_change_autonomy_test_state.getService().c_str());

  /* // | --------------------- action clients --------------------- | */

  //Waypoint Mission
  const std::string waypoint_action_client_topic = nh_.resolveName("ac/waypoint_mission");
  action_client_ptr_                             = std::make_unique<WaypointFleetManagerClient>(waypoint_action_client_topic, false);
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());

  //Coverage Mission
  const std::string coverage_action_client_topic = nh_.resolveName("ac/coverage_mission");
  coverage_action_client_ptr_                    = std::make_unique<CoveragePlannerClient>(coverage_action_client_topic, false);
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", coverage_action_client_topic.c_str());


  //Autonomy test
  const std::string autonomy_test_client_topic = nh_.resolveName("ac/autonomy_test");
  autonomy_test_client_ptr_                    = std::make_unique<AutonomyTestClient>(autonomy_test_client_topic, false);
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/autonomy_test\' -> \'%s\'", autonomy_test_client_topic.c_str());
  
  // | ------------------------- timers ------------------------- |

  timer_main_     = nh_.createTimer(ros::Rate(main_timer_rate), &IROCBridge::timerMain, this);
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

  for (auto& rh : robot_handlers_.handlers) {
    const auto& robot_name = rh.robot_name;

    if (rh.sh_general_robot_info.newMsg())
      parseGeneralRobotInfo(rh.sh_general_robot_info.getMsg(), robot_name);

    if (rh.sh_state_estimation_info.newMsg())
      parseStateEstimationInfo(rh.sh_state_estimation_info.getMsg(), robot_name);

    if (rh.sh_control_info.newMsg())
      parseControlInfo(rh.sh_control_info.getMsg(), robot_name);

    if (rh.sh_collision_avoidance_info.newMsg())
      parseCollisionAvoidanceInfo(rh.sh_collision_avoidance_info.getMsg(), robot_name);

    if (rh.sh_uav_info.newMsg())
      parseUavInfo(rh.sh_uav_info.getMsg(), robot_name);

    if (rh.sh_system_health_info.newMsg())
      parseSystemHealthInfo(rh.sh_system_health_info.getMsg(), robot_name);
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
  if (result == NULL) {
    ROS_WARN(
        "[IROCBridge]: Probably fleet_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was "
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
      robot_results[i] = {
        {"robot_name", result->robot_results[i].name},
        {"success", static_cast<bool>(result->robot_results[i].success)}, 
        {"message", result->robot_results[i].message} 
      };
    }

    // Create the main JSON object
    json json_msg = {
      {"success", static_cast<bool>(result->success)},
      {"message", result->message},
      {"robot_results", robot_results} 
    };

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
    
    json robot_json = {
        {"robot_name", rfb.name},
        {"message", rfb.message},
        {"mission_progress", rfb.mission_progress},
        {"current_goal", rfb.goal_idx},
        {"distance_to_goal", rfb.distance_to_closest_goal},
        {"goal_estimated_arrival_time", rfb.goal_estimated_arrival_time},
        {"goal_progress", rfb.goal_progress},
        {"distance_to_finish", rfb.distance_to_finish},
        {"finish_estimated_arrival_time", rfb.finish_estimated_arrival_time}
    };

    ROS_DEBUG_STREAM("[IROCBridge]: Mission feedback for robot: " << rfb.name
        << ", message: " << rfb.message
        << ", progress: " << rfb.mission_progress
        << ", current goal: " << rfb.goal_idx
        << ", distance to goal: " << rfb.distance_to_closest_goal
        << ", goal estimated arrival time: " << rfb.goal_estimated_arrival_time
        << ", goal progress: " << rfb.goal_progress
        << ", distance to finish: " << rfb.distance_to_finish
        << ", finish estimated arrival time: " << rfb.finish_estimated_arrival_time);
    
    // Add to the list at index i
    json_msgs[i] = std::move(robot_json);
  }

  // Create the main JSON message
  json json_msg = {
      {"progress", feedback->info.progress}, 
      {"mission_state", feedback->info.state}, 
      {"message", feedback->info.message}, 
      {"robots", json_msgs}
  };
  
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
    {"battery_state", {
                        {"voltage", general_robot_info->battery_state.voltage},
                        {"percentage", general_robot_info->battery_state.percentage},
                        {"wh_drained", general_robot_info->battery_state.wh_drained}
                      }},
    {"ready_to_start", general_robot_info->ready_to_start},
    {"problems_preventing_start", json::list(
        general_robot_info->problems_preventing_start.begin(),
        general_robot_info->problems_preventing_start.end())
    },
    {"errors", json::list(
        general_robot_info->errors.begin(),
        general_robot_info->errors.end())
    }
  };

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
    
    {"local_pose", {
      {"x", state_estimation_info->local_pose.position.x},
      {"y", state_estimation_info->local_pose.position.y},
      {"z", state_estimation_info->local_pose.position.z},
      {"heading", state_estimation_info->local_pose.heading}
    }},
    
    {"global_pose", {
      {"latitude", state_estimation_info->global_pose.position.x},
      {"longitude", state_estimation_info->global_pose.position.y},
      {"altitude", state_estimation_info->global_pose.position.z},
      {"heading", state_estimation_info->global_pose.heading}
    }},
    
    {"velocity", {
      {"linear", {
        {"x", state_estimation_info->velocity.linear.x},
        {"y", state_estimation_info->velocity.linear.y},
        {"z", state_estimation_info->velocity.linear.z}
      }},
      {"angular", {
        {"x", state_estimation_info->velocity.angular.x},
        {"y", state_estimation_info->velocity.angular.y},
        {"z", state_estimation_info->velocity.angular.z}
      }}
    }},
    
    {"acceleration", {
      {"linear", {
        {"x", state_estimation_info->acceleration.linear.x},
        {"y", state_estimation_info->acceleration.linear.y},
        {"z", state_estimation_info->acceleration.linear.z}
      }},
      {"angular", {
        {"x", state_estimation_info->acceleration.angular.x},
        {"y", state_estimation_info->acceleration.angular.y},
        {"z", state_estimation_info->acceleration.angular.z}
      }}
    }},

    {"running_estimators", { json::list(
      state_estimation_info->running_estimators.begin(),
      state_estimation_info->running_estimators.end())
    }},

    {"switchable_estimators", { json::list(
      state_estimation_info->switchable_estimators.begin(),
      state_estimation_info->switchable_estimators.end())
    }}
  };

  sendTelemetryJsonMessage("StateEstimationInfo", json_msg);
}
//}

/* parseControlInfo() //{ */
void IROCBridge::parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name) {
  json json_msg = {
    {"robot_name", robot_name},
    {"active_controller", control_info->active_controller},
    {"available_controllers", json::list(
        control_info->available_controllers.begin(),
        control_info->available_controllers.end())},
    {"active_tracker", control_info->active_tracker},
    {"available_trackers", json::list(
        control_info->available_trackers.begin(),
        control_info->available_trackers.end())},
    {"thrust", control_info->thrust}
  };

  sendTelemetryJsonMessage("ControlInfo", json_msg);
}
//}

/* parseCollisionAvoidanceInfo() //{ */
void IROCBridge::parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name) {
  json json_msg = {
    {"robot_name", robot_name},
    {"collision_avoidance_enabled", collision_avoidance_info->collision_avoidance_enabled},
    {"avoiding_collision", collision_avoidance_info->avoiding_collision},
    {"other_robots_visible", json::list(
        collision_avoidance_info->other_robots_visible.begin(),
        collision_avoidance_info->other_robots_visible.end())}, 
  };

  sendTelemetryJsonMessage("CollisionAvoidanceInfo", json_msg);
}
//}

/* parseUavInfo() //{ */
void IROCBridge::parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name) {
  json json_msg = {
    {"robot_name", robot_name},
    {"armed", uav_info->armed},
    {"offboard", uav_info->offboard},
    {"flight_state", uav_info->flight_state},
    {"flight_duration", uav_info->flight_duration},
    {"mass_nominal", uav_info->mass_nominal}
  };

  sendTelemetryJsonMessage("UavInfo", json_msg);
}
//}

/* parseSystemHealthInfo() //{ */
void IROCBridge::parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info, const std::string& robot_name) {
  // Create arrays for node_cpu_loads
  json node_cpu_loads = json::list();
  for (size_t i = 0; i < system_health_info->node_cpu_loads.size(); i++) {
    const auto& node_cpu_load = system_health_info->node_cpu_loads[i];

    // Create a nested array for each node_cpu_load using initializer list
    json node_entry = json::list({
        node_cpu_load.node_name,
        node_cpu_load.cpu_load
        });

    node_cpu_loads[i] = std::move(node_entry);
  }

  // Create array for available_sensors
  json available_sensors = json::list();
  for (size_t i = 0; i < system_health_info->available_sensors.size(); i++) {
    const auto& available_sensor = system_health_info->available_sensors[i];

    // Create an object for each required_sensor using initializer list
    available_sensors[i] = {
      {"name", available_sensor.name},
      {"status", available_sensor.status},
      {"ready", available_sensor.ready},
      {"rate", available_sensor.rate}
    };
  }

  // Create the main JSON object using initializer list
  json json_msg = {
    {"robot_name", robot_name},
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
    {"available_sensors", available_sensors}
  };

  sendTelemetryJsonMessage("SystemHealthInfo", json_msg);
}
//}

// --------------------------------------------------------------
// |                       helper methods                       |
// --------------------------------------------------------------
/* sendJsonMessage() //{ */
void IROCBridge::sendJsonMessage(const std::string& msg_type,json& json_msg) {
  const std::string url          = "/api/mission/" + msg_type;
  const std::string body         = json_msg.dump();
  const std::string content_type = "application/json";
  const auto        res          = http_client_->Post(url, body, content_type);

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
    case CommandType::Takeoff: return &(rh_ptr->sc_takeoff);
    case CommandType::Land: return &(rh_ptr->sc_land);
    case CommandType::Hover: return &(rh_ptr->sc_hover);
    case CommandType::Home: return &(rh_ptr->sc_land_home);
    case CommandType::Set_Origin: return &(rh_ptr->sc_set_origin);
    case CommandType::Set_SafetyBorder: return &(rh_ptr->sc_set_safety_area);
    case CommandType::Set_Obstacle: return &(rh_ptr->sc_set_obstacle);
    default: return nullptr;
  }
}

ros::ServiceClient IROCBridge::getServiceClient(const IROCBridge::Change_SvC_T service_type) {
  switch (service_type) {
    case Change_SvC_T::FleetWaypoint: return sc_change_fleet_mission_state;
    case Change_SvC_T::RobotWaypoint: return sc_change_robot_mission_state;
    case Change_SvC_T::FleetCoverage: return sc_change_coverage_mission_state;
    case Change_SvC_T::RobotCoverage: return sc_change_coverage_mission_robot_state;
    case Change_SvC_T::RobotAutonomyTest: return sc_change_autonomy_test_state;
    default: return ros::ServiceClient();
  }
}
//}

/* commandAction() method //{ */
IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string>& robot_names, const std::string& command_type) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  crow::status      status_code = crow::status::ACCEPTED; 
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
IROCBridge::action_result_t IROCBridge::commandAction(const std::vector<std::string>& robot_names, const std::string& command_type, typename Svc_T::Request req) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  crow::status      status_code = crow::status::ACCEPTED; 
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

/* toJson() method //{ */

template <typename Result>
json resultToJson(const boost::shared_ptr<const Result>& result) {
  json robot_results = json::list(); 

  for (size_t i = 0; i < result->robot_results.size(); i++) {
    robot_results[i] = {
      {"robot_name", result->robot_results[i].name},
      {"success", static_cast<bool>(result->robot_results[i].success)}, 
      {"message", result->robot_results[i].message} 
    };
  }

  // Create the main JSON object
  json json_msg = {
    {"success", static_cast<bool>(result->success)},
    {"message", result->message},
    {"robot_results", robot_results} 
  };

  return json_msg;
}
//}

/* successMissionJson() method //{ */

template <typename MissionRobot>
json successMissionJson(std::vector<MissionRobot> mission_robots) {
  json robot_results = json::list(); 

  for (size_t i = 0; i < mission_robots.size(); i++) {
    robot_results[i] = {
      {"robot_name", mission_robots[i].name},
      {"success", true }, 
      {"message", "Robot received the mission successfully"} 
    };
  }

  // Create the main JSON object
  json json_msg = {
    {"success", true},
    {"message", "Mission uploaded successfully"},
    {"robot_results", robot_results} 
  };

  return json_msg;
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
crow::response IROCBridge::pathCallback(const crow::request& req)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a path message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  std::string      robot_name = json_msg["robot_name"].s();
  std::scoped_lock lck(robot_handlers_.mtx);
  auto*            rh_ptr = findRobotHandler(robot_name, robot_handlers_);
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
  msg_path.header.stamp    = ros::Time::now();
  msg_path.header.frame_id = frame_id;
  msg_path.fly_now         = true;
  msg_path.use_heading     = false;

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
crow::response IROCBridge::setOriginCallback(const crow::request& req)
{

  ROS_INFO_STREAM("[IROCBridge]: Parsing a setOriginCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  // Get message properties
  int frame_id                       = json_msg["frame_id"].i();

  // The service supports latlon and UTM, but we we can at the moment support only latlon for IROC
  // TODO: It can be extended in the future to support UTM origin 
  mrs_msgs::ReferenceStampedSrv::Request service_request;
  service_request.header.frame_id      = "latlon_origin"; 
  service_request.header.stamp         = ros::Time::now();
  service_request.reference.position.x = json_msg["x"].d();
  service_request.reference.position.y = json_msg["y"].d();

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers

  const auto result = commandAction<mrs_msgs::ReferenceStampedSrv>(robot_names, "set_origin" , service_request);

  if (result.success) {
    ROS_INFO_STREAM("[IROCBridge]: Set origin for " << robot_names.size() << " robots.");
    world_origin_.x = json_msg["x"].d();  
    world_origin_.y = json_msg["y"].d();
  }

  return crow::response(result.status_code, result.message);
}
//}

/* setSafetyBorderCallback() method //{ */

/**
 * \brief Callback for the set safety border request. It receives a list of points and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setSafetyBorderCallback(const crow::request& req)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a setSafetyBorderCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg) {
    ROS_WARN_STREAM("[IROCBridge]: Bad json input: " << req.body);
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad json input: " + req.body + "\"}");
  }

  bool enabled = true;  // Defined default as true
                        
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
  safety_border.enabled          = enabled;
  safety_border.horizontal_frame = horizontal_frame;
  safety_border.vertical_frame   = vertical_frame;
  safety_border.points           = border_points;
  safety_border.max_z            = max_z;
  safety_border.min_z            = min_z;

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  // check that all robot names are valid and find the corresponding robot handlers
  mrs_msgs::SetSafetyBorderSrvRequest service_request;
  service_request.safety_border = safety_border;
  mrs_msgs::SetSafetyBorderSrv::Request req_srv = service_request;

  const auto result = commandAction<mrs_msgs::SetSafetyBorderSrv>(robot_names, "set_safety_border" , req_srv);

  return crow::response(result.status_code, result.message);
}
//}

/* setObstacleCallback() method //{ */

/**
 * \brief Callback for the set obstacle request. It receives a list of obstacles for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::setObstacleCallback(const crow::request& req)
{

  ROS_INFO_STREAM("[IROCBridge]: Parsing a setObstacleCallback message JSON -> ROS.");

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg)
    return crow::response(crow::status::BAD_REQUEST, "Failed to parse JSON" + req.body);

  // Get message properties
  int height_id                              = json_msg["height_id"].i();
  int max_z                                  = json_msg["max_z"].i();
  int min_z                                  = json_msg["min_z"].i();
  std::vector<crow::json::rvalue> points     = json_msg["points"].lo();

  std::string horizontal_frame = "latlon_origin";
  std::string vertical_frame;

  std::map<int, std::string> height_id_map = {
      {0, "world_origin"},
      {1, "latlon_origin"},
  };

  auto it = height_id_map.find(height_id);
  if (it != height_id_map.end())  // check if the height_id is valid (exists in the map)
    vertical_frame = it->second;

  else
    return crow::response(crow::status::BAD_REQUEST, "Unknown height_id field: " + req.body);

  if (points.empty())
    return crow::response(crow::status::BAD_REQUEST, "Empty points array: " + req.body);

  //Process points
  std::vector<mrs_msgs::Point2D> border_points;
  border_points.reserve(points.size());

  for (const auto& el : points) {
    if (!el.has("x") || !el.has("y"))
      return crow::response(crow::status::BAD_REQUEST, "Missing x or y in point: " + req.body);

    mrs_msgs::Point2D pt;
    pt.x = el["x"].d();
    pt.y = el["y"].d();

    border_points.push_back(pt);
  }

  // Logging
  ROS_INFO("[IROCBridge]: Obstacle border points size %zu ", border_points.size());

  mrs_msgs::SetObstacleSrvRequest obstacle_req;

  obstacle_req.horizontal_frame = horizontal_frame;
  obstacle_req.vertical_frame   = vertical_frame;
  obstacle_req.points           = border_points;
  obstacle_req.max_z            = max_z;
  obstacle_req.min_z            = min_z;

  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = commandAction<mrs_msgs::SetObstacleSrv>(robot_names, "set_obstacle", obstacle_req);

  return crow::response(result.status_code, result.message);
}
//}

/* waypointMissionCallback() method //{ */

/**
 * \brief Callback for the waypoint mission request. It receives a list of missions for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::waypointMissionCallback(const crow::request& req)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a waypointMissionCallback message JSON -> ROS.");
  latest_mission_type_ = "waypoint";

  try {
    crow::json::rvalue json_msg = crow::json::load(req.body);
    if (!json_msg || !json_msg.has("mission") || json_msg["mission"].t() != crow::json::type::List)
      return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad request: Failed to parse JSON or missing 'mission' key\"}");

    // Iterate over each robot in the mission
    std::vector<iroc_mission_handler::MissionGoal> mission_robots;
    for (const auto& mission_json : json_msg["mission"].lo()) {
      std::scoped_lock lck(robot_handlers_.mtx);

      std::string robot_name = mission_json["robot_name"].s();
      auto*       rh_ptr     = findRobotHandler(robot_name, robot_handlers_);
      if (!rh_ptr) {
        ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
        return crow::response(crow::status::NOT_FOUND, "{\"message\": \"Robot '" + robot_name + "' not found\"}");
      }

      // Process the action request for each robot
      iroc_mission_handler::MissionGoal mission_robot;
      mission_robot.name            = robot_name;
      mission_robot.frame_id        = mission_json["frame_id"].i();
      mission_robot.height_id       = mission_json["height_id"].i();
      mission_robot.terminal_action = mission_json["terminal_action"].i();

      // Check robot type
      bool use_z;
      auto robot_type = rh_ptr->sh_general_robot_info.getMsg()->robot_type;
      switch (robot_type) {
        case static_cast<uint8_t>(robot_type_t::MULTIROTOR): {
          ROS_INFO("[IROCBridge]: MULTIROTOR TYPE: ");
          use_z = true;
          break;
        };
        case static_cast<uint8_t>(robot_type_t::BOAT): {
          ROS_INFO("[IROCBridge]: BOAT TYPE: ");
          use_z = false;
          break;
        };
        default:
          ROS_WARN_STREAM("[IROCBridge]: Unknown robot type for robot \"" << robot_name << "\". Ignoring.");
          return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Unknown robot type for robot '" + robot_name + "'\"}");
          break;
      }

      // Process the points in the mission
      std::vector<crow::json::rvalue> points = mission_json["points"].lo();
      
      std::vector<iroc_mission_handler::Waypoint> ref_points;
      ref_points.reserve(points.size());
      for (const auto& point : points) {
        mrs_msgs::Reference ref;
        ref.position.x = point["x"].d();
        ref.position.y = point["y"].d();
        if (use_z)
          ref.position.z = point["z"].d();
        ref.heading = point["heading"].d();

        iroc_mission_handler::Waypoint waypoint;
        waypoint.reference_point = ref;
        if (point.has("subtasks")) {
          std::vector<iroc_mission_handler::Subtask> subtasks;
          for (const auto& subtask : point["subtasks"].lo()) {
            iroc_mission_handler::Subtask subtask_obj;
            subtask_obj.type = subtask["type"].i();
            if (subtask.has("parameters")) {
              subtask_obj.parameters = subtask["parameters"].s();
            }
            subtasks.push_back(subtask_obj);
          }
          waypoint.subtasks = subtasks;
        }
        ref_points.push_back(waypoint);
      }
      mission_robot.points = ref_points;
      
      // Debugging/logging
      for (const auto& point : ref_points) {
        ROS_INFO_STREAM("[IROCBridge]: Robot " << robot_name << " point: " << point.reference_point.position.x << ", " << point.reference_point.position.y << ", " << point.reference_point.position.z);
      }

      // Add the mission to the list of missions
      mission_robots.push_back(mission_robot);
    }
      
    // Validate if the action client is connected and if the action is already running
    if (!action_client_ptr_->isServerConnected()) {
      ROS_WARN_STREAM("[IROCBridge]: Action server is not connected. Check the iroc_fleet_manager node.");
      std::string msg = "Action server is not connected. Check iroc_fleet_manager node.\n";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }
    else if (!action_client_ptr_->getState().isDone()) {
      ROS_WARN_STREAM("[IROCBridge]: Mission is already running. Terminate the previous one, or wait until it is finished.");
      std::string msg = "Mission is already running. Terminate the previous one, or wait until it is finished";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }

    // Send the action goal to the fleet manager
    FleetManagerActionServerGoal action_goal;
    action_goal.robots = mission_robots;

    action_client_ptr_->sendGoal(
        action_goal,
        [this](const auto& state, const auto& result) {
          missionDoneCallback<iroc_fleet_manager::WaypointFleetManagerResult>(state, result);
        },
        [this]() {
          missionActiveCallback();
        },
        [this](const auto& feedback) {
          missionFeedbackCallback<iroc_fleet_manager::WaypointFleetManagerFeedback>(feedback);
        }
    );

    // Waiting in the case the trajectories are rejected. We can better wait will the state is pending
    ros::Duration(mission_robots.size() * 1.0).sleep();

    if (action_client_ptr_->getState().isDone()) {  // If the action is done, the action finished instantly
      auto result = action_client_ptr_->getResult();
      auto json   = resultToJson(result); 
      const auto message = result->message;
      ROS_WARN("[IROCBridge]: %s", message.c_str());
      return crow::response(crow::status::BAD_REQUEST, json);
    }
    else {
      ROS_INFO("[IROCBridge]: Mission received successfully");
      auto json = successMissionJson(mission_robots);
      return crow::response(crow::status::CREATED, json);
    }
  }
  catch (const std::exception& e) {
    ROS_WARN_STREAM("[IROCBridge]: Failed to parse JSON from message: " << e.what());
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Failed to parse JSON from message: " + std::string(e.what()) + "\"}");
  }
}
//}

/* coverageMissionCallback() method //{ */

/**
 * \brief Callback for the coverage mission request. It receives a list of missions for each robot and sends them to the fleet manager.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::coverageMissionCallback(const crow::request& req)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a coverageMissionCallback message JSON -> ROS.");
  latest_mission_type_ = "coverage";

  try {
    crow::json::rvalue json_msg = crow::json::load(req.body);
    if (!json_msg || !json_msg.has("robots") || json_msg["robots"].t() != crow::json::type::List
        || !json_msg.has("search_area") || json_msg["search_area"].t() != crow::json::type::List )
      return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad request: Failed to parse JSON or missing 'mission' key\"}");

    // Get message properties currently we are setting the same parameters for all robots
    std::vector<crow::json::rvalue> robots      = json_msg["robots"].lo();
    std::vector<crow::json::rvalue> search_area = json_msg["search_area"].lo();
    int height                                  = json_msg["height"].i();
    int height_id                               = json_msg["height_id"].i();
    int terminal_action                         = json_msg["terminal_action"].i();

    // To store the robots
    std::vector<iroc_fleet_manager::CoverageMissionRobot> mission_robots;

    //Get the robot names
    std::vector<std::string> robot_names;
    robot_names.reserve(robots.size());
    for (const auto& robot : robots) {
      robot_names.push_back(robot.s());
    }

    for (const auto& robot : robot_names) {
      std::scoped_lock lck(robot_handlers_.mtx);
      iroc_fleet_manager::CoverageMissionRobot mission_robot;
      auto*       rh_ptr     = findRobotHandler(robot, robot_handlers_);
      if (!rh_ptr) {
        ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot << "\" not found. Ignoring.");
        return crow::response(crow::status::NOT_FOUND, "{\"message\": \"Robot '" + robot + "' not found\"}");
      }
      // Get the current robot position, to use it as the starting point of the coverage mission
      auto robot_global_pose = rh_ptr->sh_state_estimation_info.getMsg()->global_pose;
      auto robot_local_pose = rh_ptr->sh_state_estimation_info.getMsg()->local_pose;
      // Fill the mission robot struct
      mission_robot.name            = robot;
      mission_robot.global_position.x      = robot_global_pose.position.x;
      mission_robot.global_position.y      = robot_global_pose.position.y;
      mission_robot.global_position.z      = robot_global_pose.position.z;
      mission_robot.local_position.x       = robot_local_pose.position.x;
      mission_robot.local_position.y       = robot_local_pose.position.y;
      mission_robot.local_position.z       = robot_local_pose.position.z;
      mission_robot.frame_id               = iroc_mission_handler::MissionGoal::FRAME_ID_LATLON;
      mission_robot.height_id              = height_id;
      mission_robot.height                 = height;
      mission_robot.terminal_action        = terminal_action;
      mission_robots.push_back(mission_robot);
    }

    //Get the search area points
    std::vector<mrs_msgs::Point2D> polygon_points;
    polygon_points.reserve(search_area.size());

    for (const auto& el : search_area) {
      mrs_msgs::Point2D pt;
      pt.x = el["x"].d();
      pt.y = el["y"].d();

      polygon_points.push_back(pt);
    }

    ROS_INFO("[IROCBridge]: Polygon points size %zu ", polygon_points.size());

    // Validate if the action client is connected and if the action is already running
    if (!coverage_action_client_ptr_->isServerConnected()) {
      ROS_WARN_STREAM("[IROCBridge]: Action server is not connected. Check the iroc_fleet_manager node.");
      std::string msg = "Action server is not connected. Check iroc_fleet_manager node.\n";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }
    else if (!coverage_action_client_ptr_->getState().isDone()) {
      ROS_WARN_STREAM("[IROCBridge]: Mission is already running. Terminate the previous one, or wait until it is finished.");
      std::string msg = "Mission is already running. Terminate the previous one, or wait until it is finished";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }

    // Send the action goal to the fleet manager
    coverageMissionActionServerGoal action_goal;

    if (world_origin_.x == 0.0 && world_origin_.y == 0.0) {
      ROS_WARN_STREAM("[IROCBridge]: World origin is not set.");
      std::string msg = "World origin is not set.";
      return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
    }

    action_goal.mission.latlon_origin = world_origin_;
    action_goal.mission.robots        = mission_robots; 
    action_goal.mission.search_area   = polygon_points;

    coverage_action_client_ptr_->sendGoal(
        action_goal,
        [this](const auto& state, const auto& result) {
          missionDoneCallback<iroc_fleet_manager::CoverageMissionResult>(state, result);
        },
        [this]() {
          missionActiveCallback();
        },
        [this](const auto& feedback) {
          missionFeedbackCallback<iroc_fleet_manager::CoverageMissionFeedback>(feedback);
        }
    );

    // Waiting in the case the trajectories are rejected. We can better wait will the state is pending
    // bool finished_before_timeout = coverage_action_client_ptr_->waitForResult(ros::Duration(3.0));
    // ROS_INFO("[IROCBridge]: Finished before timeout: %d", finished_before_timeout);
    ros::Duration(mission_robots.size() * 1.0).sleep();

    if (coverage_action_client_ptr_->getState().isDone()) {
      auto result = coverage_action_client_ptr_->getResult();
      const auto message = result->message;
      auto json   = resultToJson(result); 
      ROS_WARN("[IROCBridge]: %s", message.c_str());
      return crow::response(crow::status::BAD_REQUEST, json);
    }
    else {
      ROS_INFO("[IROCBridge]: Mission received successfully");
      auto json = successMissionJson(mission_robots);
      return crow::response(crow::status::CREATED, json);
    }
  }
  catch (const std::exception& e) {
    ROS_WARN_STREAM("[IROCBridge]: Failed to parse JSON from message: " << e.what());
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Failed to parse JSON from message: " + std::string(e.what()) + "\"}");
  }
}
//}

/* autonomyTestCallback() method //{ */

/**
 * \brief Callback for the autonomy test request. It receives the request for a specific robot the and sends them to the fleet manager autonomy test component.
 *
 * \param req Crow request
 * \return res Crow response
 */
crow::response IROCBridge::autonomyTestCallback(const crow::request& req)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a autonomyTestCallback message JSON -> ROS.");
  latest_mission_type_ = "autonomy_test";

  crow::json::rvalue json_msg = crow::json::load(req.body);
  if (!json_msg || !json_msg.has("robot_name"))
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Bad request: Failed to parse JSON or missing 'robot_name' key\"}");

  // Get message properties
  std::string  robot_name = json_msg["robot_name"].s();
  int segment_length      = json_msg["segment_length"].i();

  std::scoped_lock lck(robot_handlers_.mtx);

  auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);

  if (!rh_ptr) {
    ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
    return crow::response(crow::status::BAD_REQUEST, "{\"message\": \"Robot \"" + robot_name + "\" not found, ignoring\"}");
  }

  auto robot_type = rh_ptr->sh_general_robot_info.getMsg()->robot_type;

  switch (robot_type) {
    case static_cast<uint8_t>(robot_type_t::MULTIROTOR): {
      ROS_INFO("[IROCBridge]: MULTIROTOR TYPE: ");
      bool use_heading = false;
      break;
    };

    case static_cast<uint8_t>(robot_type_t::BOAT): {
      ROS_INFO("[IROCBridge]: BOAT TYPE: ");
      bool use_heading = false;
      break;
    };

    default:
      break;
  }

  //We are using an array of robots as input, as we are using the waypoint mission structure that expects an array of robots.
  //Kept for simplicity to avoid changing the previous structure that supports multiple robots.
  //As we decide that autonomy test will be for one drone at a time, we are only adding the specified robot for the autonomy test.
  std::vector<iroc_fleet_manager::AutonomyTestRobot> mission_robots;
  iroc_fleet_manager::AutonomyTestRobot mission_robot;
  mission_robot.name           = robot_name;
  mission_robot.segment_length = segment_length;
  mission_robots.push_back(mission_robot);

  AutonomyTestActionServerGoal action_goal;
  action_goal.robots = mission_robots;

  if (!autonomy_test_client_ptr_->isServerConnected()) {
    std::string msg = "Action server is not connected. Check iroc_fleet_manager node.\n";
    ROS_WARN_STREAM("[IROCBridge]: Action server is not connected. Check the iroc_fleet_manager node.");
    return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
  }

  if (!autonomy_test_client_ptr_->getState().isDone()) {
    ROS_WARN_STREAM("[IROCBridge]: Mission is already running. Terminate the previous one, or wait until it is finished.");
    std::string msg = "Mission is already running. Terminate the previous one, or wait until it is finished";
    return crow::response(crow::status::CONFLICT, "{\"message\": \"" + msg + "\"}");
  }

  autonomy_test_client_ptr_->sendGoal(
        action_goal,
        [this](const auto& state, const auto& result) {
          missionDoneCallback<iroc_fleet_manager::AutonomyTestResult>(state, result);
        },
        [this]() {
          missionActiveCallback();
        },
        [this](const auto& feedback) {
          missionFeedbackCallback<iroc_fleet_manager::AutonomyTestFeedback>(feedback);
        }
    );

  ros::Duration(mission_robots.size() * 1.0).sleep();

  if (autonomy_test_client_ptr_->getState().isDone()) {  // If the action is done, the action finished instantly
    auto result = autonomy_test_client_ptr_->getResult();
    const auto message = result->message;
    ROS_WARN("[IROCBridge]: %s", message.c_str());
    return crow::response(crow::status::SERVICE_UNAVAILABLE, "{\"message\": \"" + message + "\"}");
  }
  else {
    ROS_INFO("[IROCBridge]: Mission received successfully");
    return crow::response(crow::status::CREATED, "{\"message\": \"Mission received successfully\"}");
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
crow::response IROCBridge::changeFleetMissionStateCallback(const crow::request& req, const std::string& type)
{
  std::scoped_lock lck(robot_handlers_.mtx);

  // Input validation
  if (type != "start" && type != "stop" && type != "pause")
    return crow::response(crow::status::NOT_FOUND);

  mrs_msgs::String ros_srv;
  ros_srv.request.value = type;

  ros::ServiceClient service_client;
  auto it = change_type_map_.find(latest_mission_type_);
  if (it != change_type_map_.end()) 
    service_client = getServiceClient(it->second); 

  const auto resp = callService<mrs_msgs::String>(service_client, ros_srv.request);
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
crow::response IROCBridge::changeRobotMissionStateCallback(const crow::request& req, const std::string& robot_name, const std::string& type)
{
  std::scoped_lock lck(robot_handlers_.mtx);

  // Input validation
  if (type != "start" && type != "stop" && type != "pause")
    return crow::response(crow::status::NOT_FOUND);
  if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(), [&robot_name](const auto& rh) { return rh.robot_name == robot_name; }))
    return crow::response(crow::status::NOT_FOUND, "Robot not found");

  json json_msg;

  iroc_fleet_manager::ChangeRobotMissionStateSrv ros_srv;
  ros_srv.request.robot_name = robot_name;
  ros_srv.request.type       = type;

  ros::ServiceClient service_client;
  auto it = change_robot_type_map_.find(latest_mission_type_);
  if (it != change_robot_type_map_.end()) 
    service_client = getServiceClient(it->second); 

  const auto resp = callService<iroc_fleet_manager::ChangeRobotMissionStateSrv>(service_client, ros_srv.request);
  if (!resp.success) {
    json_msg["message"] = "Call was not successful with message: " + resp.message;
    ROS_WARN_STREAM("[IROCBridge]: " << json_msg["message"].dump());

    return crow::response(crow::status::INTERNAL_SERVER_ERROR, json_msg.dump());
  }
  else {
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
crow::response IROCBridge::commandCallback(const crow::request& req,
    const std::string & command_type,
    std::optional<std::string> robot_name)
{
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
crow::response IROCBridge::availableRobotsCallback([[maybe_unused]] const crow::request& req)
{
  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  json json_msg;
  json_msg["robot_names"] = robot_names;

  return crow::response(crow::status::ACCEPTED, json_msg.dump());
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
void IROCBridge::remoteControlCallback(crow::websocket::connection& conn, const std::string& data, bool is_binary)
{
  // Convert and check if the received data is a valid JSON
  crow::json::rvalue json_data = crow::json::load(data);
  if (!json_data || !json_data.has("command") || !json_data.has("data")) {
    ROS_WARN_STREAM("[IROCBridge]: Failed to parse JSON from websocket message: " << data);
    conn.send_text("{\"error\": \"Failed to parse JSON or missing 'command'/'data'\"}");
    return;
  }

  json json_response;
  std::string        command = json_data["command"].s();
  if (command == "message") {
    ROS_INFO_STREAM("[IROCBridge]: Received message from " << conn.get_remote_ip() << ": " << json_data["data"].s());
    conn.send_text("{\"status\": \"Ok, received message\"}");
  }
  else if (command == "move") {
    std::scoped_lock lck(robot_handlers_.mtx);

    // Robot id validation
    if (!json_data.has("robot_name")) {
      ROS_WARN_STREAM("[IROCBridge]: Missing robot_id in websocket message: " << data);
      conn.send_text("{\"error\": \"Missing robot_id\"}");
      return;
    }

    std::string robot_name = json_data["robot_name"].s();
    if (!std::any_of(robot_handlers_.handlers.begin(), robot_handlers_.handlers.end(), [robot_name](const robot_handler_t& rh) {
          return rh.robot_name == robot_name;
        })) {
      ROS_WARN_STREAM("[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
      conn.send_text("{\"error\": \"Robot not found\"}");
      return;
    }

    crow::json::rvalue movement_data = json_data["data"];

    mrs_msgs::VelocityReferenceStampedSrvRequest req;
    req.reference.header.frame_id = "fcu_untilted";

    req.reference.reference.velocity.x       = movement_data["x"].d() * max_linear_speed_;
    req.reference.reference.velocity.y       = movement_data["y"].d() * max_linear_speed_;
    req.reference.reference.velocity.z       = movement_data["z"].d() * max_linear_speed_;
    req.reference.reference.heading_rate     = movement_data["heading"].d() * max_heading_rate_;
    req.reference.reference.use_heading_rate = true;

    auto* robot_handler_ptr = findRobotHandler(robot_name, robot_handlers_);
    auto  res               = callService<mrs_msgs::VelocityReferenceStampedSrv>(robot_handler_ptr->sc_velocity_reference, req);

    if (res.success) {
      json_response["ok"]      = true;
      json_response["message"] = "Movement command sent";
    }
    else {
      json_response["ok"]      = false;
      json_response["message"] = "Failed to send movement command: " + res.message;
    }
  }
  else {
    ROS_WARN_STREAM("[IROCBridge]: Unknown command in websocket message: " << command);
    json_response["ok"]      = false;
    json_response["message"] = "Unknown command";
  }
  conn.send_text(json_response.dump());
}
//}

}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
