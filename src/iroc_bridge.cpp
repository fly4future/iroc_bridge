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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

/* custom msgs of MRS group */
#include <mrs_msgs/Path.h>

#include <mrs_msgs/String.h>
#include <iroc_fleet_manager/ChangeRobotMissionStateSrv.h>

#include <mrs_msgs/SetSafetyBorderSrv.h>
#include <mrs_msgs/SetSafetyBorderSrvRequest.h>
#include <mrs_msgs/SetSafetyBorderSrvResponse.h>

#include <mrs_msgs/SetObstacleSrv.h>
#include <mrs_msgs/SetObstacleSrvRequest.h>
#include <mrs_msgs/SetObstacleSrvResponse.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>

#include <mrs_mission_manager/waypointMissionAction.h>
#include <iroc_fleet_manager/WaypointFleetManagerAction.h>
#include <iroc_fleet_manager/WaypointMissionRobot.h>
#include <iroc_fleet_manager/WaypointMissionInfo.h>

#include "iroc_bridge/json_var_parser.h"
#include <mrs_robot_diagnostics/parsing_functions.h>

#include <unistd.h>
#include <iostream>

//}

namespace iroc_bridge
{

using json = nlohmann::json;

using vec3_t = Eigen::Vector3d;
using vec4_t = Eigen::Vector4d;

using namespace actionlib;

//to remove
/* typedef SimpleActionClient<mrs_mission_manager::waypointMissionAction>               MissionManagerClient; */
typedef SimpleActionClient<iroc_fleet_manager::WaypointFleetManagerAction> WaypointFleetManagerClient;
//to remove
/* typedef mrs_mission_manager::waypointMissionGoal                                     ActionServerGoal; */
typedef iroc_fleet_manager::WaypointFleetManagerGoal                       FleetManagerActionServerGoal;

typedef mrs_robot_diagnostics::robot_type_t     robot_type_t;

/* class IROCBridge //{ */

class IROCBridge : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  std::thread                      th_http_srv_;
  httplib::Server                  http_srv_;
  std::unique_ptr<httplib::Client> http_client_;

  struct result_t
  {
    bool        success;
    std::string message;
  };

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
    ros::ServiceClient sc_set_safety_area;
    ros::ServiceClient sc_set_obstacle;
    ros::ServiceClient sc_mission_activation;
    ros::ServiceClient sc_mission_pausing;

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

  // | ----------------- action client callbacks ---------------- |

  void waypointMissionActiveCallback(const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robots);
  void waypointMissionDoneCallback(const SimpleClientGoalState& state, const iroc_fleet_manager::WaypointFleetManagerResultConstPtr& result,
                                 const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robots);
  void waypointMissionFeedbackCallback(const iroc_fleet_manager::WaypointFleetManagerFeedbackConstPtr& result, const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robot);

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string& robot_name);
  void parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string& robot_name);
  void parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name);
  void parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name);
  void parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name);
  void parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr uav_info, const std::string& robot_name);

  void             sendJsonMessage(const std::string& msg_type, const json& json_msg);
  robot_handler_t* findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers);

  result_t takeoffAction(const std::vector<std::string>& robot_names);
  result_t landAction(const std::vector<std::string>& robot_names);
  result_t hoverAction(const std::vector<std::string>& robot_names);
  result_t landHomeAction(const std::vector<std::string>& robot_names);
  result_t setSafetyBorderAction(const std::vector<std::string>& robot_names, const mrs_msgs::SafetyBorder& msg); 
  result_t setObstacleAction(const std::vector<std::string>& robot_names, const mrs_msgs::SetObstacleSrvRequest& req); 

  void pathCallback(const httplib::Request&, httplib::Response& res);
  void setSafetyBorderCallback(const httplib::Request&, httplib::Response& res);
  void setObstacleCallback(const httplib::Request&, httplib::Response& res);
  void waypointMissionCallback(const httplib::Request&, httplib::Response& res);
  void changeFleetMissionStateCallback(const httplib::Request&, httplib::Response& res);
  void changeRobotMissionStateCallback(const httplib::Request&, httplib::Response& res);
  void takeoffCallback(const httplib::Request&, httplib::Response& res);
  void takeoffAllCallback(const httplib::Request&, httplib::Response& res);
  void hoverCallback(const httplib::Request&, httplib::Response& res);
  void hoverAllCallback(const httplib::Request&, httplib::Response& res);
  void landCallback(const httplib::Request&, httplib::Response& res);
  void landHomeCallback(const httplib::Request&, httplib::Response& res);
  void landAllCallback(const httplib::Request&, httplib::Response& res);
  void landHomeAllCallback(const httplib::Request&, httplib::Response& res);
  void availableRobotsCallback(const httplib::Request&, httplib::Response& res);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

  std::thread th_death_check_;
  void        routine_death_check();

  bool active_border_callback_;

  std::unique_ptr<WaypointFleetManagerClient> action_client_ptr_;
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

  const auto main_timer_rate    = param_loader.loadParam2<double>("main_timer_rate");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  const auto url         = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");

  const auto robot_names = param_loader.loadParam2<std::vector<std::string>>("network/robot_names");
  
  //Remove ground-station hostname from robot names
  
  std::string hostname_str(hostname.data());
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

  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  const httplib::Server::Handler hdlr_set_path = std::bind(&IROCBridge::pathCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_path", hdlr_set_path);

  const httplib::Server::Handler hdlr_set_safety_area =
      std::bind(&IROCBridge::setSafetyBorderCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_safety_border", hdlr_set_safety_area);

  const httplib::Server::Handler hdlr_set_obstacle =
      std::bind(&IROCBridge::setObstacleCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_obstacle", hdlr_set_obstacle);

  const httplib::Server::Handler hdlr_set_waypoint_mission =
      std::bind(&IROCBridge::waypointMissionCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_waypoint_mission", hdlr_set_waypoint_mission);

  const httplib::Server::Handler hdlr_change_fleet_mission_state =
      std::bind(&IROCBridge::changeFleetMissionStateCallback, this, std::placeholders::_1, std::placeholders::_2);
  //Using raw string for the creation of the endpoint
  http_srv_.Post(R"(/fleet/mission/(start|stop|pause))", hdlr_change_fleet_mission_state);

  const httplib::Server::Handler hdlr_change_robot_mission =
      std::bind(&IROCBridge::changeRobotMissionStateCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post(R"(/robots/(\w+)/mission/(start|stop|pause))", hdlr_change_robot_mission);

  const httplib::Server::Handler hdlr_takeoff = std::bind(&IROCBridge::takeoffCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/takeoff", hdlr_takeoff);

  const httplib::Server::Handler hdlr_hover = std::bind(&IROCBridge::hoverCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/hover", hdlr_hover);

  const httplib::Server::Handler hdlr_hover_all = std::bind(&IROCBridge::hoverAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/hover_all", hdlr_hover_all);

  const httplib::Server::Handler hdlr_takeoff_all = std::bind(&IROCBridge::takeoffAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/takeoff_all", hdlr_takeoff_all);

  const httplib::Server::Handler hdlr_land = std::bind(&IROCBridge::landCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/land", hdlr_land);

  const httplib::Server::Handler hdlr_land_home = std::bind(&IROCBridge::landHomeCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/land_home", hdlr_land_home);

  const httplib::Server::Handler hdlr_land_all = std::bind(&IROCBridge::landAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/land_all", hdlr_land_all);

  const httplib::Server::Handler hdlr_land_home_all = std::bind(&IROCBridge::landHomeAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/land_home_all", hdlr_land_home_all);

  const httplib::Server::Handler hdlr_available_robots = std::bind(&IROCBridge::availableRobotsCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/available_robots", hdlr_available_robots);

  th_http_srv_ = std::thread([&]() { http_srv_.listen(url, server_port); });
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

      //To remove
      robot_handler.sc_mission_activation = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/mission_activation"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_activation\' -> \'%s\'", robot_handler.sc_mission_activation.getService().c_str());

      //To remove
      robot_handler.sc_mission_pausing = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/mission_pausing"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/mission_pausing\' -> \'%s\'", robot_handler.sc_mission_pausing.getService().c_str());

      robot_handler.sc_set_safety_area = nh_.serviceClient<mrs_msgs::SetSafetyBorderSrv>("/" + robot_name + nh_.resolveName("svc/set_safety_area"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/set_safety_area\' -> \'%s\'", robot_handler.sc_mission_pausing.getService().c_str());

      robot_handler.sc_set_obstacle = nh_.serviceClient<mrs_msgs::SetObstacleSrv>("/" + robot_name + nh_.resolveName("svc/set_obstacle"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/set_obstacle\' -> \'%s\'", robot_handler.sc_mission_pausing.getService().c_str());


      // | ----------------------- publishers ----------------------- |
      robot_handler.pub_path = nh_.advertise<mrs_msgs::Path>("/" + robot_name + nh_.resolveName("out/path"), 2);
      ROS_INFO("[IROCBridge]: Created publisher on topic \'out/path\' -> \'%s\'", robot_handler.pub_path.getTopic().c_str());

      // move is necessary because copy construction of the subscribe handlers is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }

  sc_change_fleet_mission_state = nh_.serviceClient<mrs_msgs::String>(nh_.resolveName("svc/change_fleet_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_fleet_mission_state\' -> \'%s\'", sc_change_fleet_mission_state.getService().c_str());

  sc_change_robot_mission_state = nh_.serviceClient<iroc_fleet_manager::ChangeRobotMissionStateSrv>(nh_.resolveName("svc/change_robot_mission_state"));
  ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc_server/change_robot_mission_state\' -> \'%s\'", sc_change_robot_mission_state.getService().c_str());

  /* // | --------------------- action clients --------------------- | */
  const std::string waypoint_action_client_topic = nh_.resolveName("ac/waypoint_mission");
  action_client_ptr_                = std::make_unique<WaypointFleetManagerClient>(waypoint_action_client_topic, false);
  ROS_INFO("[IROCBridge]: Created action client on topic \'ac/waypoint_mission\' -> \'%s\'", waypoint_action_client_topic.c_str());

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
// |                  acition client callbacks                  |
// --------------------------------------------------------------

/* waypointMissionActiveCallback //{ */

void IROCBridge::waypointMissionActiveCallback(const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robots) {

  ROS_INFO_STREAM("[IROCBridge]: Waypoint Mission Action server for robots: ");

  for (const auto& robot : robots) {
    ROS_INFO_STREAM(robot);
  }
}

//}

/* waypointMissionDoneCallback //{ */

void IROCBridge::waypointMissionDoneCallback(const SimpleClientGoalState& state, const iroc_fleet_manager::WaypointFleetManagerResultConstPtr& result,
   const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robots) {

  if (result == NULL) {
    ROS_WARN("[IROCBridge]: Probably mission_manager died, and action server connection was lost!, reconnection is not currently handled, if mission manager was restarted need to upload a new mission!");
    const json json_msg = {
      {"mission_result", "Mission manager died in ongoing mission"},
      {"mission_success", false},
    };
    sendJsonMessage("WaypointMissionDone", json_msg);
  } else {
    if (result->success) {
      ROS_INFO_STREAM("[IROCBridge]: Mission Action server finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
    } else {
      ROS_ERROR_STREAM("[IROCBridge]: Mission Action server finished with state: \"" << state.toString() << "\". Result message is: \""
          << result->message << "\"");
    }

    const json json_msg = {
      {"mission_result", result->message},
      {"mission_success", result->success},
    };
    sendJsonMessage("WaypointMissionDone", json_msg);
  }
}

//}

/* waypointMissionFeedbackCallback //{ */

void IROCBridge::waypointMissionFeedbackCallback(const iroc_fleet_manager::WaypointFleetManagerFeedbackConstPtr& feedback, const std::vector<iroc_fleet_manager::WaypointMissionRobot>& robots) {
 
  /* ROS_INFO_STREAM("[IROCBridge]: Feedback from " << feedback->info.message << "State: " << feedback->info.state << "Progress: " << feedback->info.progress); */ 

  auto robots_feedback = feedback->info.robots_feedback; 
  std::vector<json> json_msgs;
 
  //Collect each robot feedback and create a json for each
  for (const auto& rfb : robots_feedback) {
    json json_msg = {
      {"robot_name", rfb.name},
      {"message", rfb.message},
      {"mission_progress", rfb.mission_progress},
      {"current_goal", rfb.goal_idx},
      {"distance_to_goal", rfb.distance_to_closest_goal},
      {"goal_estimated_arrival_time", rfb.goal_estimated_arrival_time},
      {"goal_progress", rfb.goal_progress},
      {"distance_to_finish", rfb.distance_to_finish},
      {"finish_estimated_arrival_time",rfb.finish_estimated_arrival_time },
    };
    json_msgs.emplace_back(json_msg);
  }

  const json json_msg = {
    {"progress", feedback->info.progress },
    {"mission_state", feedback->info.state},
    {"message", feedback->info.message},
    {"robots", json_msgs}
  };

  sendJsonMessage("WaypointMissionFeedback", json_msg);
}

//}

// --------------------------------------------------------------
// |                 parsing and output methods                 |
// --------------------------------------------------------------

/* parseGeneralRobotInfo() //{ */

void IROCBridge::parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string& robot_name) {
  const json json_msg = {
      {"robot_name", general_robot_info->robot_name},
      {"robot_type", general_robot_info->robot_type},
      {"battery_state",
       {
           {"voltage", general_robot_info->battery_state.voltage},
           {"percentage", general_robot_info->battery_state.percentage},
           {"wh_drained", general_robot_info->battery_state.wh_drained},
       }},
      {"ready_to_start", general_robot_info->ready_to_start},
      {"problems_preventing_start", general_robot_info->problems_preventing_start},
      {"errors", general_robot_info->errors},
  };
  sendJsonMessage("GeneralRobotInfo", json_msg);
}

//}

/* parseStateEstimationInfo() //{ */

void IROCBridge::parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string& robot_name) {
  const json json_msg = {
      {"robot_name", robot_name},
      {"estimation_frame", state_estimation_info->header.frame_id},
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
      {"above_ground_level_height", state_estimation_info->above_ground_level_height},
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
      {"current_estimator", state_estimation_info->current_estimator},
      {"running_estimators", state_estimation_info->running_estimators},
      {"switchable_estimators", state_estimation_info->switchable_estimators}};
  sendJsonMessage("StateEstimationInfo", json_msg);
}

//}

/* parseControlInfo() //{ */

void IROCBridge::parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string& robot_name) {
  const json json_msg = {
      {"robot_name", robot_name},
      {"active_controller", control_info->active_controller},
      {"available_controllers", control_info->available_controllers},
      {"active_tracker", control_info->active_tracker},
      {"available_trackers", control_info->available_trackers},
      {"thrust", control_info->thrust},
  };
  sendJsonMessage("ControlInfo", json_msg);
}

//}

/* parseCollisionAvoidanceInfo() //{ */

void IROCBridge::parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string& robot_name) {
  const json json_msg = {
      {"robot_name", robot_name},
      {"collision_avoidance_enabled", collision_avoidance_info->collision_avoidance_enabled},
      {"avoiding_collision", collision_avoidance_info->avoiding_collision},
      {"other_robots_visible", collision_avoidance_info->other_robots_visible},
  };
  sendJsonMessage("CollisionAvoidanceInfo", json_msg);
}

//}

/* parseUavInfo() //{ */

void IROCBridge::parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string& robot_name) {
  const json json_msg = {
      {"robot_name", robot_name},
      {"armed", uav_info->armed},
      {"offboard", uav_info->offboard},
      {"flight_state", uav_info->flight_state},
      {"flight_duration", uav_info->flight_duration},
      {"mass_nominal", uav_info->mass_nominal},
      {"mass_estimate", uav_info->mass_estimate},
  };
  sendJsonMessage("UavInfo", json_msg);
}

//}

/* parseSystemHealthInfo() //{ */

void IROCBridge::parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info, const std::string& robot_name) {
  json node_cpu_loads;
  for (const auto& node_cpu_load : system_health_info->node_cpu_loads)
    node_cpu_loads.emplace_back(json::array({node_cpu_load.node_name, node_cpu_load.cpu_load}));

  json required_sensors;
  for (const auto& required_sensor : system_health_info->required_sensors)
    required_sensors.emplace_back(json{
        {"name", required_sensor.name},
        {"status", required_sensor.status},
        {"ready", required_sensor.ready},
        {"rate", required_sensor.rate},
    });

  const json json_msg = {
      {"robot_name", robot_name},
      {"cpu_load", system_health_info->cpu_load},
      {"free_ram", system_health_info->free_ram},
      {"total_ram", system_health_info->total_ram},
      {"free_hdd", system_health_info->free_hdd},
      {"node_cpu_loads", node_cpu_loads},
      {"hw_api_rate", system_health_info->hw_api_rate},
      {"control_manager_rate", system_health_info->control_manager_rate},
      {"state_estimation_rate", system_health_info->state_estimation_rate},
      {"gnss_uncertainty", system_health_info->gnss_uncertainty},
      {"mag_strength", system_health_info->mag_strength},
      {"mag_uncertainty", system_health_info->mag_uncertainty},
      {"required_sensors", required_sensors},
  };
  sendJsonMessage("SystemHealthInfo", json_msg);
}

//}

// --------------------------------------------------------------
// |                       helper methods                       |
// --------------------------------------------------------------

/* sendJsonMessage() //{ */

void IROCBridge::sendJsonMessage(const std::string& msg_type, const json& json_msg) {
  const std::string url          = "/api/robot/telemetry/" + msg_type;
  const std::string body         = json_msg.dump();
  const std::string content_type = "application/json";
  const auto        res          = http_client_->Post(url, body, content_type);

  if (res)
    ROS_INFO_STREAM_THROTTLE(1.0, res->status << ": " << res->body);
  else
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to send PATCH request to address \"" << url << "\": " << to_string(res.error()));

  return;
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
      ROS_ERROR_STREAM_THROTTLE(1.0, "Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
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

// --------------------------------------------------------------
// |                   Action implementations                   |
// --------------------------------------------------------------

/* takeoffAction() method //{ */
IROCBridge::result_t IROCBridge::takeoffAction(const std::vector<std::string>& robot_names) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling takeoff.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_takeoff);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* hoverAction() method //{ */
IROCBridge::result_t IROCBridge::hoverAction(const std::vector<std::string>& robot_names) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling hover.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_hover);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* landAction() method //{ */
IROCBridge::result_t IROCBridge::landAction(const std::vector<std::string>& robot_names) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling land.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_land);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* landHomeAction() method //{ */
IROCBridge::result_t IROCBridge::landHomeAction(const std::vector<std::string>& robot_names) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling land home.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_land_home);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* setSafetyBorderAction() method //{ */
IROCBridge::result_t IROCBridge::setSafetyBorderAction(const std::vector<std::string>& robot_names, const mrs_msgs::SafetyBorder& msg) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  mrs_msgs::SetSafetyBorderSrvRequest req;
  req.safety_border = msg;

  ROS_INFO_STREAM_THROTTLE(1.0, "Calling set safety border service.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<mrs_msgs::SetSafetyBorderSrv>(rh_ptr->sc_set_safety_area, req);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* setObstacleAction() method //{ */
IROCBridge::result_t IROCBridge::setObstacleAction(const std::vector<std::string>& robot_names, const mrs_msgs::SetObstacleSrvRequest& req) {
  std::scoped_lock lck(robot_handlers_.mtx);

  bool              everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers

  ROS_INFO_STREAM_THROTTLE(1.0, "Calling set safety obstacle service.");
  for (const auto& robot_name : robot_names) {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr) {
      const auto resp = callService<mrs_msgs::SetObstacleSrv>(rh_ptr->sc_set_obstacle, req);
      if (!resp.success) {
        ss << "Call for robot \"" << robot_name << "\" was not successful with message: " << resp.message << "\n";
        everything_ok = false;
      }
    } else {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

// --------------------------------------------------------------
// |                     REST API callbacks                     |
// --------------------------------------------------------------

/* pathCallback() method //{ */
void IROCBridge::pathCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a path message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string robot_name, frame_id;
  json        points;
  const auto  succ = parse_vars(json_msg, {{"robot_name", &robot_name}, {"frame_id", &frame_id}, {"points", &points}});
  if (!succ)
    return;

  if (!points.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
    return;
  }

  mrs_msgs::Path msg_path;
  msg_path.points.reserve(points.size());
  bool use_heading = false;
  for (const auto& el : points) {
    mrs_msgs::Reference ref;
    const auto          succ = parse_vars(el, {{"x", &ref.position.x}, {"y", &ref.position.y}, {"z", &ref.position.z}});
    if (!succ)
      return;

    if (el.contains("heading")) {
      ref.heading = el.at("heading");
      use_heading = true;
    }
    msg_path.points.push_back(ref);
  }
  msg_path.header.stamp    = ros::Time::now();
  msg_path.header.frame_id = frame_id;
  msg_path.fly_now         = true;
  msg_path.use_heading     = use_heading;

  std::stringstream ss;
  std::scoped_lock  lck(robot_handlers_.mtx);
  auto*             rh_ptr = findRobotHandler(robot_name, robot_handlers_);
  if (!rh_ptr) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
    ss << "robot \"" << robot_name << "\" not found, ignoring";
    res.status = httplib::StatusCode::BadRequest_400;
    res.body   = ss.str();
    return;
  }

  rh_ptr->pub_path.publish(msg_path);

  ss << "set a path with " << points.size() << " length for robot \"" << robot_name << "\"";
  ROS_INFO_STREAM("[IROCBridge]: Set a path with " << points.size() << " length.");
  res.status = httplib::StatusCode::Accepted_202;
  res.body   = ss.str();
}
//}

/* setSafetyBorderCallback() method //{ */
void IROCBridge::setSafetyBorderCallback(const httplib::Request& req, httplib::Response& res) {

  active_border_callback_ = true;

  ROS_INFO_STREAM("[IROCBridge]: Parsing a setSafetyBorderCallback message JSON -> ROS.");
  std::stringstream ss;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad json input: " << e.what();
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  std::string units;
  int         origin_x;
  int         origin_y;
  //Defined default as true
  bool        enabled = true;
  //Defined to be latlon origin by default, as for the UI perspective it makes more sense
  std::string horizontal_frame = "latlon_origin";
  std::string vertical_frame;
  int         height_id;
  json        points;
  int         max_z;
  int         min_z;

  std::map<int, std::string> height_id_map = {
    {0, "world_origin"},
    {1, "latlon_origin"},
  };
  
  const auto  succ = parse_vars(json_msg, {
      {"height_id", &height_id}, 
      {"points", &points}, 
      {"max_z", &max_z}, 
      {"min_z", &min_z}
   });
  
  if (!succ)
    return;

  if (!points.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad points input: Expected an array "; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  //Map the received id and save it into the corresponding vertical frame string
  auto it = height_id_map.find(height_id);

  if (it != height_id_map.end()) {
    vertical_frame = it->second;
  } else {
    ROS_ERROR_STREAM("[IROCBridge]: Unknown height_id: " << height_id);
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Unknown height_id field"; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  std::vector<mrs_msgs::Point2D> border_points;
  border_points.reserve(points.size());

  for (const auto& el : points) {
    mrs_msgs::Point2D pt;
    const auto          succ = parse_vars(el, {{"x", &pt.x}, {"y", &pt.y}});
    if (!succ)
      return;
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

  const auto result = setSafetyBorderAction(robot_names, safety_border);

  if (result.success) {
    res.status = httplib::StatusCode::Created_201;
    ss << "Safety zone created successfully."; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
  } else {
    res.status = httplib::StatusCode::InternalServerError_500;
    ss << "Safety zone creation failed."; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
  }
  active_border_callback_ = false;
}
//}

/* setObstacleCallback() method //{ */
void IROCBridge::setObstacleCallback(const httplib::Request& req, httplib::Response& res) {

  std::stringstream ss;

  if (active_border_callback_) {
    return;
  }

  ROS_INFO_STREAM("[IROCBridge]: Parsing a setObstacleCallback message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad mission input: Expected an array"; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  //Defined to be latlon origin by default, as for the UI perspective it makes more sense
  std::string horizontal_frame = "latlon_origin";
  std::string vertical_frame;
  json        points;
  int         height_id;
  int         max_z;
  int         min_z;

  std::map<int, std::string> height_id_map = {
    {0, "world_origin"},
    {1, "latlon_origin"},
  };

  const auto  succ = parse_vars(json_msg, {
      {"height_id", &height_id}, 
      {"points", &points}, 
      {"max_z", &max_z}, 
      {"min_z", &min_z}
   });
  
  if (!succ)
    return;

  if (!points.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad points input: Expected an array"; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  //Map the received id and save it into the corresponding vertical frame string
  auto it = height_id_map.find(height_id);

  if (it != height_id_map.end()) {
    vertical_frame = it->second;
  } else {
    ROS_ERROR_STREAM("[IROCBridge]: Unknown height_id: " << height_id);
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Unknown height_id field"; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  std::vector<mrs_msgs::Point2D> border_points;
  border_points.reserve(points.size());

  for (const auto& el : points) {
    mrs_msgs::Point2D pt;
    const auto succ = parse_vars(el, {{"x", &pt.x}, {"y", &pt.y}});
    if (!succ)
      return;
    border_points.push_back(pt);
  }

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
  
  const auto result = setObstacleAction(robot_names, obstacle_req);

  if (result.success) {
    res.status = httplib::StatusCode::Created_201;
    ss << result.message; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
  } else {
    res.status = httplib::StatusCode::InternalServerError_500;
    ss << result.message; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
  }
}
//}

/* waypointMissionCallback() method //{ */
void IROCBridge::waypointMissionCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a waypointMissionCallback message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  std::stringstream ss;

  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad json input: " << e.what();
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  json        mission;
  const auto  succ = parse_vars(json_msg, {{"mission", &mission}});
  if (!succ)
    return;

  if (!mission.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad mission input: Expected an array.");
    res.status = httplib::StatusCode::BadRequest_400;
    ss << "Bad mission input: Expected an array"; 
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  //Process the robots assigned into the mission 
  std::vector<iroc_fleet_manager::WaypointMissionRobot> mission_robots;
  mission_robots.reserve(mission.size());
 
  for (const auto& robot : mission ) {

    int         frame_id;
    int         height_id;
    int         terminal_action;
    std::string robot_name;
    json        points;
    const auto  succ = parse_vars(robot, 
        {{"robot_name", &robot_name}, {"frame_id", &frame_id}, {"height_id", &height_id}, {"points", &points}, {"terminal_action", &terminal_action}});

    if (!succ)
      return;
    if (!points.is_array()) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
      res.status = httplib::StatusCode::BadRequest_400;
      ss << "Bad points input: Expected an array"; 
      json json_response_msg = {{"message", ss.str()}};
      res.set_content(json_response_msg.dump(), "application/json");
      return;
    }

    std::stringstream ss;
    std::scoped_lock  lck(robot_handlers_.mtx);
    auto*             rh_ptr = findRobotHandler(robot_name, robot_handlers_);

    if (!rh_ptr) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
      res.status = httplib::StatusCode::BadRequest_400;
      ss << "robot \"" << robot_name << "\" not found, ignoring";
      json json_response_msg = {{"message", ss.str()}};
      res.set_content(json_response_msg.dump(), "application/json");
      return;
    }

    auto robot_type =  rh_ptr->sh_general_robot_info.getMsg()->robot_type;
    std::vector<mrs_msgs::Reference> ref_points;
    ref_points.reserve(points.size());

    switch (robot_type){
      case static_cast<uint8_t>(robot_type_t::MULTIROTOR): {
        ROS_INFO("[IROCBridge]: MULTIROTOR TYPE: ");
        bool use_heading = false;
        for (const auto& el : points) {
            mrs_msgs::Reference ref;
            const auto          succ = parse_vars(el, {{"x", &ref.position.x}, {"y", &ref.position.y}, {"z", &ref.position.z}, {"heading", &ref.heading}});
            if (!succ)
              return;
            ref_points.push_back(ref);
        }
        break;
      };

      case static_cast<uint8_t>(robot_type_t::BOAT): {
        ROS_INFO("[IROCBridge]: BOAT TYPE: ");
        bool use_heading = false;
        for (const auto& el : points) {
          mrs_msgs::Reference ref;
          const auto          succ = parse_vars(el, {{"x", &ref.position.x}, {"y", &ref.position.y}, {"heading", &ref.heading}});
          if (!succ)
            return;
          ref_points.push_back(ref);
        }
        break;
      };

      default:
        break;
    }
  
    iroc_fleet_manager::WaypointMissionRobot mission_robot;
    mission_robot.name            = robot_name;
    mission_robot.frame_id        = frame_id;
    mission_robot.height_id       = height_id; 
    mission_robot.points          = ref_points;
    mission_robot.terminal_action = terminal_action;
    //Save the individual robot mission information
    mission_robots.push_back(mission_robot);
  }

  //Debugging/logging
  for (const auto& robot : mission_robots) {
   ROS_INFO_STREAM("[IROCBridge]: Assigning robot " << robot.name << " into mission");
  }

  FleetManagerActionServerGoal action_goal;
  action_goal.robots = mission_robots;

  if (!action_client_ptr_->isServerConnected()) {
    ss << "Action server is not connected. Check iroc_fleet_manager node.\n";
    ROS_ERROR_STREAM("[IROCBridge]: Action server is not connected. Check the iroc_fleet_manager node.");
    res.status = httplib::StatusCode::InternalServerError_500;
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  if (!action_client_ptr_->getState().isDone()) {
    ROS_ERROR_STREAM("[IROCBridge]: Mission is already running. Terminate the previous one, or wait until it is finished.");
    ss << "Mission is already running. Terminate the previous one, or wait until it is finished";
    res.status = httplib::StatusCode::Conflict_409;
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
    return;
  }

  action_client_ptr_->sendGoal(
      action_goal, std::bind(&IROCBridge::waypointMissionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, mission_robots),
      std::bind(&IROCBridge::waypointMissionActiveCallback, this, mission_robots ),
      std::bind(&IROCBridge::waypointMissionFeedbackCallback, this, std::placeholders::_1, mission_robots));

  ss << "Mission processed successfully";
    res.status = httplib::StatusCode::Created_201;
    json json_response_msg = {{"message", ss.str()}};
    res.set_content(json_response_msg.dump(), "application/json");
 }
//}

/* changeFleetMissionStateCallback() method //{ */
void IROCBridge::changeFleetMissionStateCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a changeFleetMissionStateCallback message JSON -> ROS.");

  auto type = req.matches[1];

  std::stringstream ss;
  mrs_msgs::String ros_srv;
  ros_srv.request.value = type; 

  const auto resp = callService<mrs_msgs::String>(sc_change_fleet_mission_state, ros_srv.request);
  if (!resp.success) { 
    ss << "Mission start service was not successful with message: " << resp.message << "\n";
    res.status = httplib::StatusCode::InternalServerError_500;
  } else {
    res.status = httplib::StatusCode::Accepted_202;
    ss << "Mission started successfully";
  }
  json json_msg = {{"message", ss.str()}};
  res.set_content(json_msg.dump(), "application/json");
}
//}

/* changeRobotMissionStateCallback() method //{ */
void IROCBridge::changeRobotMissionStateCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a changeRobotMissionStateCallback message JSON -> ROS.");

  auto robot_name = req.matches[1];
  auto type = req.matches[2];

  std::stringstream ss;
  iroc_fleet_manager::ChangeRobotMissionStateSrv ros_srv;
  ros_srv.request.robot_name = robot_name;
  ros_srv.request.type       = type;

  const auto resp = callService<iroc_fleet_manager::ChangeRobotMissionStateSrv>(sc_change_robot_mission_state, ros_srv.request);
  if (!resp.success) { 
    ss << "Mission start service was not successful with message: " << resp.message << "\n";
    res.status = httplib::StatusCode::InternalServerError_500;
  } else {
    res.status = httplib::StatusCode::Accepted_202;
    ss << "Mission started successfully";
  }
  json json_msg = {{"message", ss.str()}};
  res.set_content(json_msg.dump(), "application/json");
}
//}

/* takeoffCallback() method //{ */
void IROCBridge::takeoffCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a takeoff message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string type;

  json       robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = takeoffAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* hoverCallback() method //{ */
void IROCBridge::hoverCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a hover message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string type;

  json       robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = hoverAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* landCallback() method //{ */
void IROCBridge::landCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a land message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string type;

  json       robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = landAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* landHomeCallback() method //{ */
void IROCBridge::landHomeCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Parsing a land home message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
  json json_msg;
  try {
    json_msg = json::parse(req.body);
  }
  catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad json input: " << e.what());
    return;
  }

  std::string type;

  json       robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array()) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = landHomeAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* takeoffAllCallback() method //{ */
void IROCBridge::takeoffAllCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Received takeoff all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = takeoffAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* hoverAllCallback() method //{ */
void IROCBridge::hoverAllCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Received hover all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = hoverAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* landAllCallback() method //{ */
void IROCBridge::landAllCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Received land all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = landAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* landHomeAllCallback() method //{ */
void IROCBridge::landHomeAllCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Received landHome all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = landHomeAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body   = result.message;
}
//}

/* availableRobotsCallback() method //{ */
void IROCBridge::availableRobotsCallback(const httplib::Request& req, httplib::Response& res) {
  ROS_INFO_STREAM("[IROCBridge]: Received request for available robots");
  auto json_robots = json::array();
  for (const auto& rh : robot_handlers_.handlers)
    json_robots.push_back(rh.robot_name);

  const json json_msg = {
      {"robot_names", json_robots},
  };

  res.body   = json_msg.dump();
  res.status = httplib::StatusCode::Accepted_202;
}
//}

}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
