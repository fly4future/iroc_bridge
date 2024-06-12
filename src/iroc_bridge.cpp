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
#include <mrs_msgs/Path.h>

#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>

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

  struct result_t
  {
    bool success;
    std::string message;
  };

  struct robot_handler_t
  {
    std::string robot_name;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo>        sh_general_robot_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo>     sh_state_estimation_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo>             sh_control_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo>  sh_collision_avoidance_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo>                 sh_uav_info;
    mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo>        sh_system_health_info;

    ros::ServiceClient sc_arm;
    ros::ServiceClient sc_offboard;
    ros::ServiceClient sc_land;

    ros::Publisher pub_path;
  };

  // | ---------------------- ROS subscribers --------------------- |

  struct robot_handlers_t
  {
    std::mutex mtx;
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void timerMain(const ros::TimerEvent &event);

  // | ------------------ Additional functions ------------------ |

  void parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string &robot_name);
  void parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string &robot_name);
  void parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string &robot_name);
  void parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string &robot_name);
  void parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string &robot_name);
  void parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr uav_info, const std::string &robot_name);

  void sendJsonMessage(const std::string& msg_type, const json& json_msg);
  robot_handler_t* findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers);

  result_t takeoffAction(const std::vector<std::string>& robot_names);
  result_t landAction(const std::vector<std::string>& robot_names);

  void pathCallback(const httplib::Request&, httplib::Response& res);
  void waypointMissionCallback(const httplib::Request&, httplib::Response& res);
  void changeMissionStateCallback(const httplib::Request&, httplib::Response& res);
  void takeoffCallback(const httplib::Request&, httplib::Response& res);
  void takeoffAllCallback(const httplib::Request&, httplib::Response& res);
  void landCallback(const httplib::Request&, httplib::Response& res);
  void landAllCallback(const httplib::Request&, httplib::Response& res);

  // some helper method overloads
  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc, typename Svc_T::Request req);

  template <typename Svc_T>
  result_t callService(ros::ServiceClient& sc);

  result_t callService(ros::ServiceClient& sc, const bool val);

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

  const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");
  const auto no_message_timeout = param_loader.loadParam2<ros::Duration>("no_message_timeout");

  const auto url = param_loader.loadParam2<std::string>("url");
  const auto client_port = param_loader.loadParam2<int>("client_port");
  const auto server_port = param_loader.loadParam2<int>("server_port");

  const auto robot_names = param_loader.loadParam2<std::vector<std::string>>("network/robot_names");

  if (!param_loader.loadedSuccessfully())
  {
    ROS_ERROR("[IROCBridge]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------- HTTP REST API callbacks ---------------- |

  http_client_ = std::make_unique<httplib::Client>(url, client_port);

  const httplib::Server::Handler hdlr_set_path = std::bind(&IROCBridge::pathCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_path", hdlr_set_path);

  const httplib::Server::Handler hdlr_set_waypoint_mission = std::bind(&IROCBridge::waypointMissionCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/set_waypoint_mission", hdlr_set_waypoint_mission);

  const httplib::Server::Handler hdlr_change_mission_state = std::bind(&IROCBridge::changeMissionStateCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/change_mission_state", hdlr_change_mission_state);

  const httplib::Server::Handler hdlr_takeoff = std::bind(&IROCBridge::takeoffCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/takeoff", hdlr_takeoff);

  const httplib::Server::Handler hdlr_takeoff_all = std::bind(&IROCBridge::takeoffAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/takeoff_all", hdlr_takeoff_all);

  const httplib::Server::Handler hdlr_land = std::bind(&IROCBridge::landCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Post("/land", hdlr_land);

  const httplib::Server::Handler hdlr_land_all = std::bind(&IROCBridge::landAllCallback, this, std::placeholders::_1, std::placeholders::_2);
  http_srv_.Get("/land_all", hdlr_land_all);

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

  // populate the robot handlers vector
  {
    std::scoped_lock lck(robot_handlers_.mtx);
    
    robot_handlers_.handlers.reserve(robot_names.size());
    for (const auto& robot_name : robot_names)
    {
      robot_handler_t robot_handler;
      robot_handler.robot_name = robot_name;
    
      const std::string general_robot_info_topic_name = "/" + robot_name + nh_.resolveName("in/general_robot_info");
      robot_handler.sh_general_robot_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::GeneralRobotInfo>(shopts, general_robot_info_topic_name);
    
      const std::string state_estimation_info_topic_name = "/" + robot_name + nh_.resolveName("in/state_estimation_info");
      robot_handler.sh_state_estimation_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::StateEstimationInfo>(shopts, state_estimation_info_topic_name);
    
      const std::string control_info_topic_name = "/" + robot_name + nh_.resolveName("in/control_info");
      robot_handler.sh_control_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::ControlInfo>(shopts, control_info_topic_name);
    
      const std::string collision_avoidance_info_topic_name = "/" + robot_name + nh_.resolveName("in/collision_avoidance_info");
      robot_handler.sh_collision_avoidance_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::CollisionAvoidanceInfo>(shopts, collision_avoidance_info_topic_name);
    
      const std::string uav_info_topic_name = "/" + robot_name + nh_.resolveName("in/uav_info");
      robot_handler.sh_uav_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::UavInfo>(shopts, uav_info_topic_name);
    
      const std::string system_health_info_topic_name = "/" + robot_name + nh_.resolveName("in/system_health_info");
      robot_handler.sh_system_health_info = mrs_lib::SubscribeHandler<mrs_robot_diagnostics::SystemHealthInfo>(shopts, system_health_info_topic_name);

      robot_handler.sc_arm = nh_.serviceClient<std_srvs::SetBool>("/" + robot_name + nh_.resolveName("svc/arm"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/arm\' -> \'%s\'", robot_handler.sc_arm.getService().c_str());

      robot_handler.sc_offboard = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/offboard"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/offboard\' -> \'%s\'", robot_handler.sc_offboard.getService().c_str());

      robot_handler.sc_land = nh_.serviceClient<std_srvs::Trigger>("/" + robot_name + nh_.resolveName("svc/land"));
      ROS_INFO("[IROCBridge]: Created ServiceClient on service \'svc/land\' -> \'%s\'", robot_handler.sc_land.getService().c_str());


      // | ----------------------- publishers ----------------------- |
      robot_handler.pub_path = nh_.advertise<mrs_msgs::Path>("/" + robot_name + nh_.resolveName("out/path"), 2);
      ROS_INFO("[IROCBridge]: Created publisher on topic \'out/path\' -> \'%s\'", robot_handler.pub_path.getTopic().c_str());

      // move is necessary because copy construction of the subscribe handlers is deleted due to mutexes
      robot_handlers_.handlers.emplace_back(std::move(robot_handler));
    }
  }

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
  std::scoped_lock lck(robot_handlers_.mtx);

  for (auto& rh : robot_handlers_.handlers)
  {
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
// |                 parsing and output methods                 |
// --------------------------------------------------------------

/* parseGeneralRobotInfo() //{ */

void IROCBridge::parseGeneralRobotInfo(mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info, const std::string &robot_name)
{
  const json json_msg =
  {
    {"robot_name", general_robot_info->robot_name},
    {"robot_type", general_robot_info->robot_type},
    {"battery_state",
      {
        {"voltage", general_robot_info->battery_state.voltage},
        {"percentage", general_robot_info->battery_state.percentage},
        {"wh_drained", general_robot_info->battery_state.wh_drained},
      }
    },
    {"ready_to_start", general_robot_info->ready_to_start},
    {"problem_preventing_start", general_robot_info->problem_preventing_start},
  };
  sendJsonMessage("GeneralRobotInfo", json_msg);
}

//}

/* parseStateEstimationInfo() //{ */

void IROCBridge::parseStateEstimationInfo(mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info, const std::string &robot_name)
{
  const json json_msg = 
  {
    {"robot_name", robot_name},
    {"estimation_frame", state_estimation_info->header.frame_id},
    {"local_pose",
        {
          {"x", state_estimation_info->local_pose.position.x},
          {"y", state_estimation_info->local_pose.position.y},
          {"z", state_estimation_info->local_pose.position.z},
          {"heading", state_estimation_info->local_pose.heading}
        }
    },
    {"global_pose",
        {
          {"latitude", state_estimation_info->global_pose.position.x},
          {"longitude", state_estimation_info->global_pose.position.y},
          {"altitude", state_estimation_info->global_pose.position.z},
          {"heading", state_estimation_info->global_pose.heading}
        }
    },
    {"above_ground_level_height", state_estimation_info->above_ground_level_height},
    {"velocity",
        {
          {"linear",
              {
                  {"x", state_estimation_info->velocity.linear.x},
                  {"y", state_estimation_info->velocity.linear.y},
                  {"z", state_estimation_info->velocity.linear.z}
              }
          },
          {"angular",
              {
                  {"x", state_estimation_info->velocity.angular.x},
                  {"y", state_estimation_info->velocity.angular.y},
                  {"z", state_estimation_info->velocity.angular.z}
              }
          }
        }
    },
    {"acceleration",
        {
          {"linear",
              {
                  {"x", state_estimation_info->acceleration.linear.x},
                  {"y", state_estimation_info->acceleration.linear.y},
                  {"z", state_estimation_info->acceleration.linear.z}
              }
          },
          {"angular",
              {
                  {"x", state_estimation_info->acceleration.angular.x},
                  {"y", state_estimation_info->acceleration.angular.y},
                  {"z", state_estimation_info->acceleration.angular.z}
              }
          }
        }
    },
    {"current_estimator", state_estimation_info->current_estimator},
    {"running_estimators", state_estimation_info->running_estimators},
    {"switchable_estimators", state_estimation_info->switchable_estimators}
  };
  sendJsonMessage("StateEstimationInfo", json_msg);
}

//}

/* parseControlInfo() //{ */

void IROCBridge::parseControlInfo(mrs_robot_diagnostics::ControlInfo::ConstPtr control_info, const std::string &robot_name)
{
  const json json_msg =
  {
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

void IROCBridge::parseCollisionAvoidanceInfo(mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info, const std::string &robot_name)
{
  const json json_msg =
  {
    {"robot_name", robot_name},
    {"collision_avoidance_enabled", collision_avoidance_info->collision_avoidance_enabled},
    {"avoiding_collision", collision_avoidance_info->avoiding_collision},
    {"other_robots_visible", collision_avoidance_info->other_robots_visible},
  };
  sendJsonMessage("CollisionAvoidanceInfo", json_msg);
}

//}

/* parseUavInfo() //{ */

void IROCBridge::parseUavInfo(mrs_robot_diagnostics::UavInfo::ConstPtr uav_info, const std::string &robot_name)
{
  const json json_msg =
  {
    {"robot_name", robot_name},
    {"armed",   uav_info->armed},
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

void IROCBridge::parseSystemHealthInfo(mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info, const std::string &robot_name)
{
  json node_cpu_loads;
  for (const auto& node_cpu_load : system_health_info->node_cpu_loads)
    node_cpu_loads.emplace_back(json::array({
          node_cpu_load.node_name,
          node_cpu_load.cpu_load
          }));

  json required_sensors;
  for (const auto& required_sensor : system_health_info->required_sensors)
    required_sensors.emplace_back(json
        {
          {"name", required_sensor.name},
          {"status", required_sensor.status},
          {"ready", required_sensor.ready},
          {"rate", required_sensor.rate},
        });

  const json json_msg =
  {
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

void IROCBridge::sendJsonMessage(const std::string& msg_type, const json& json_msg)
{
  const std::string url = "/api/robot/telemetry/" + msg_type;
  const std::string body = json_msg.dump();
  const std::string content_type = "application/x-www-form-urlencoded";
  const auto res = http_client_->Post(url, body, content_type);
  
  if (res)
    ROS_INFO_STREAM_THROTTLE(1.0, res->status << ": " << res->body);
  else
    ROS_WARN_STREAM_THROTTLE(1.0, "Failed to send PATCH request to address \"" << url << "\": " << to_string(res.error()));

  return;
}

//}

/* callService() //{ */

template <typename Svc_T>
IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc, typename Svc_T::Request req)
{
  typename Svc_T::Response res;
  if (sc.call(req, res))
  {
    ROS_INFO_STREAM("Called service \"" << sc.getService() << "\" with response \"" << res.message << "\".");
    return {true, res.message};
  }
  else
  {
    const std::string msg = "Failed to call service \"" + sc.getService() + "\".";
    ROS_WARN_STREAM(msg);
    return {false, msg};
  }
}

template <typename Svc_T>
IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc)
{
  return callService<Svc_T>(sc, {});
}

IROCBridge::result_t IROCBridge::callService(ros::ServiceClient& sc, const bool val)
{
  using svc_t = std_srvs::SetBool;
  svc_t::Request req;
  req.data = val;
  return callService<svc_t>(sc, req);
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

/* findRobotHandler() method //{ */
IROCBridge::robot_handler_t* IROCBridge::findRobotHandler(const std::string& robot_name, robot_handlers_t& robot_handlers)
{
  for (auto& rh : robot_handlers_.handlers)
  {
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
IROCBridge::result_t IROCBridge::takeoffAction(const std::vector<std::string>& robot_names)
{
  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  std::vector<robot_handler_t*> robot_handlers;
  robot_handlers.reserve(robot_names.size());
  for (const auto& robot_name : robot_names)
  {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr)
      robot_handlers.push_back(rh_ptr);
    else
    {
      ss << "robot \"" << robot_name << "\" not found, skipping\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Skipping.");
      everything_ok = false;
    }
  }

  // firstly, arm the vehicles
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling arm.");
  for (const auto& rh_ptr : robot_handlers)
  {
    const auto resp = callService(rh_ptr->sc_arm, true);
    if (!resp.success)
    {
      ss << "failed to arm \"" << rh_ptr->robot_name << "\"\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Failed to call arm service of robot \"" << rh_ptr->robot_name << "\".");
      everything_ok = false;
    }
  }

  const ros::Duration wait_after_arm(1.0);
  ROS_INFO_STREAM_THROTTLE(1.0, "Waiting " << wait_after_arm.toSec() << "s after arming before swithing to offboard mode.");
  wait_after_arm.sleep();

  ROS_INFO_STREAM_THROTTLE(1.0, "Calling takeoff by switching to the offboard mode");
  for (const auto& rh_ptr : robot_handlers)
  {
    const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_offboard);
    if (!resp.success)
    {
      ss << "failed to switch \"" << rh_ptr->robot_name << "\" to offobard\n";
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Failed to call offboard of robot \"" << rh_ptr->robot_name << "\".");
      everything_ok = false;
    }
  }

  return {everything_ok, ss.str()};
}
//}

/* landAction() method //{ */
IROCBridge::result_t IROCBridge::landAction(const std::vector<std::string>& robot_names)
{
  std::scoped_lock lck(robot_handlers_.mtx);

  bool everything_ok = true;
  std::stringstream ss;
  ss << "Result:\n";

  // check that all robot names are valid and find the corresponding robot handlers
  ROS_INFO_STREAM_THROTTLE(1.0, "Calling land.");
  for (const auto& robot_name : robot_names)
  {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (rh_ptr != nullptr)
      const auto resp = callService<std_srvs::Trigger>(rh_ptr->sc_land);
    else
    {
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
void IROCBridge::pathCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a path message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
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

  std::string robot_name, frame_id;
  json points;
  const auto succ = parse_vars(json_msg, {{"robot_name", &robot_name}, {"frame_id", &frame_id}, {"points", &points}});
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

  std::stringstream ss;
  std::scoped_lock lck(robot_handlers_.mtx);
  auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
  if (!rh_ptr)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
    ss << "robot \"" << robot_name << "\" not found, ignoring";
    res.status = httplib::StatusCode::BadRequest_400;
    res.body = ss.str();
    return;
  }

  rh_ptr->pub_path.publish(msg_path);

  ss << "set a path with " << points.size() << " length for robot \"" << robot_name << "\"";
  ROS_INFO_STREAM("[IROCBridge]: Set a path with " << points.size() << " length.");
  res.status = httplib::StatusCode::Accepted_202;
  res.body = ss.str();
}
//}

/* waypointMissionCallback() method //{ */
void IROCBridge::waypointMissionCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a waypointMissionCallback message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
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
  std::string robot_name;
  std::string terminal_action;
  json points;
  const auto succ = parse_vars(json_msg, {{"robot_name", &robot_name}, {"frame_id", &frame_id}, {"points", &points}, {"terminal_action", &terminal_action}});
  if (!succ)
    return;

  if (!points.is_array())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad points input: Expected an array.");
    return;
  }

  std::stringstream ss;
  std::scoped_lock lck(robot_handlers_.mtx);
  auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
  if (!rh_ptr)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
    ss << "robot \"" << robot_name << "\" not found, ignoring";
    res.status = httplib::StatusCode::BadRequest_400;
    res.body = ss.str();
    return;
  }

  const int print_indent = 2;
  ROS_INFO("[IROCBridge]: msg: \n%s", json_msg.dump(print_indent).c_str());
  res.status = httplib::StatusCode::Accepted_202;
}
//}

/* changeMissionStateCallback() method //{ */
void IROCBridge::changeMissionStateCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a changeMissionStateCallback message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
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

  std::string type;

  json robot_names;
  const auto succ = parse_vars(json_msg, {{"type", &type}, {"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  std::stringstream ss;
  std::scoped_lock lck(robot_handlers_.mtx);
  for (const auto& robot_name : robot_names)
  {
    auto* rh_ptr = findRobotHandler(robot_name, robot_handlers_);
    if (!rh_ptr)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Robot \"" << robot_name << "\" not found. Ignoring.");
      ss << "robot \"" << robot_name << "\" not found, ignoring";
      res.status = httplib::StatusCode::BadRequest_400;
      res.body = ss.str();
      return;
    }
  }

  const int print_indent = 2;
  ROS_INFO("[IROCBridge]: msg: \n%s", json_msg.dump(print_indent).c_str());
  res.status = httplib::StatusCode::Accepted_202;
}
//}

/* takeoffCallback() method //{ */
void IROCBridge::takeoffCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a takeoff message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
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

  std::string type;

  json robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = takeoffAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body = result.message;
}
//}

/* takeoffAllCallback() method //{ */
void IROCBridge::takeoffAllCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Received takeoff all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = takeoffAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body = result.message;
}
//}

/* landCallback() method //{ */
void IROCBridge::landCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Parsing a landCallback message JSON -> ROS.");
  res.status = httplib::StatusCode::UnprocessableContent_422;
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

  std::string type;

  json robot_names;
  const auto succ = parse_vars(json_msg, {{"robot_names", &robot_names}});
  if (!succ)
    return;

  if (!robot_names.is_array())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[IROCBridge]: Bad \'robot_names\' input: Expected an array.");
    return;
  }

  const auto result = landAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body = result.message;
}
//}

/* landAllCallback() method //{ */
void IROCBridge::landAllCallback(const httplib::Request& req, httplib::Response& res)
{
  ROS_INFO_STREAM("[IROCBridge]: Received land all request.");
  std::scoped_lock lck(robot_handlers_.mtx);

  std::vector<std::string> robot_names;
  robot_names.reserve(robot_handlers_.handlers.size());
  for (const auto& rh : robot_handlers_.handlers)
    robot_names.push_back(rh.robot_name);

  const auto result = landAction(robot_names);

  res.status = httplib::StatusCode::Accepted_202;
  res.body = result.message;
}
//}

}  // namespace iroc_bridge

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(iroc_bridge::IROCBridge, nodelet::Nodelet);
