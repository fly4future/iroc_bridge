#pragma once

/**
 * \file iroc_bridge.hpp
 * \brief Bidirectional translator between the web frontend (HTTP/WebSocket) and ROS 2.
 *
 * IROCBridge is a ROS 2 composable component that exposes a Crow HTTP server
 * (default port 8080, configurable via the 'iroc_bridge/server_port' ROS parameter) 
 * and WebSocket endpoints for real-time telemetry, mission feedback, and remote control.
 * It also acts as an HTTP client to push notifications to the backend 
 * (default port 8000, configurable via 'iroc_bridge/client_port').
 *
 * Key data flows:
 * - **Telemetry (ROS -> Web):** A main timer polls per-robot subscriber handlers and
 *   converts MRS diagnostic messages to JSON, broadcasting them via WebSocket.
 * - **Commands (Web -> ROS):** HTTP POST endpoints map to ROS service calls (takeoff,
 *   land, hover, set_origin, set_safety_border, etc.), forwarded to robots or fleet manager.
 * - **Remote control (Web -> ROS):** A WebSocket endpoint receives joystick-style velocity
 *   commands and forwards them to the per-robot velocity_reference service.
 * - **Missions (Web <-> ROS):** HTTP endpoints for uploading, querying, and controlling
 *   fleet missions, backed by fleet manager service calls and an action client.
 */

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
#include <string>
#include <memory>

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

/**
 * \brief HTTP/WebSocket-to-ROS 2 bridge (ROS 2 composable component).
 *
 * Translates between the web frontend and the ROS 2 system. Hosts a Crow HTTP
 * server with REST endpoints and WebSocket channels for telemetry streaming,
 * mission feedback, and remote control. Communicates with robots and the fleet
 * manager via ROS service clients and an action client.
 */
class IROCBridge : public mrs_lib::Node {
public:
  IROCBridge(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;   ///< Callback group for subscribers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;      ///< Callback group for service servers.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;      ///< Callback group for service clients.
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;  ///< Callback group for timers.

  /** \brief Loads config, creates HTTP server/client, sets up subscribers, service clients, WebSocket endpoints, and timers. */
  void initialize(void);
  /** \brief Graceful shutdown handler. */
  void shutdown();

  // | ---------------------- HTTP server & client --------------------- |

  std::thread                  th_http_srv_;  ///< Thread running the Crow HTTP server.
  crow::App<crow::CORSHandler> http_srv_;     ///< Crow HTTP server with CORS middleware (port 8080).

  std::unique_ptr<httplib::Client> http_client_; ///< HTTP client for posting to the backend (port 8000).

  using result_t = iroc_common::result_t;

  /**
   * \brief Return type for HTTP command actions.
   *
   * - `success`: Whether the underlying ROS service call(s) succeeded.
   * - `message`: Human-readable status or error description.
   * - `status_code`: HTTP status code to return to the client.
   */
  struct action_result_t
  {
    bool         success;
    std::string  message;
    crow::status status_code;
  };

  // | ---------------------- Command types --------------------- |

  /**
   * \brief Supported robot command types mapped from HTTP endpoint strings.
   */
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

  /**
   * \brief Mission state change service variants used to select the correct service call.
   */
  enum class Change_SvC_T
  {
    FleetWaypoint,
    RobotWaypoint,
    FleetCoverage,
    RobotCoverage,
    RobotAutonomyTest
  };

  // | ---------------------- ROS parameters ------------------ |

  double max_linear_speed_; ///< Maximum linear velocity for remote control commands (m/s).
  double max_heading_rate_; ///< Maximum heading angular rate for remote control commands (rad/s).

  // | ---------------------- Per-robot handlers --------------------- |

  /**
   * \brief Per-robot ROS interface bundle: subscribers for telemetry topics and
   * service clients for control commands.
   *
   * One instance per robot. Subscribers poll diagnostic topics (GeneralRobotInfo,
   * StateEstimationInfo, ControlInfo, etc.) and service clients issue commands
   * (takeoff, land, hover, set_origin, set_safety_area, set_obstacle, velocity_reference).
   */
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

    mrs_lib::PublisherHandler<mrs_msgs::msg::Path> pub_path; ///< Path publisher for visualization.
  };

  /**
   * \brief Thread-safe container for all per-robot handler bundles.
   */
  struct robot_handlers_t
  {
    std::recursive_mutex         mtx;      ///< Guards access to the handlers vector.
    std::vector<robot_handler_t> handlers;
  } robot_handlers_;

  /// Maps command name strings ("takeoff", "land", etc.) to the corresponding service client member pointer.
  std::map<std::string, mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> robot_handler_t::*> trigger_command_handlers_ = {
      {"takeoff", &robot_handler_t::sc_takeoff},
      {"land", &robot_handler_t::sc_land},
      {"hover", &robot_handler_t::sc_hover},
      {"home", &robot_handler_t::sc_land_home}};

  // | ----------------------- Main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  /** \brief Polls all robot subscribers for new messages and triggers telemetry parsing/broadcasting. */
  void                       timerMain();

  // | ----------------------- Fleet manager service clients ----------------------- |

  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeFleetMissionStateSrv> sc_change_fleet_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::ChangeRobotMissionStateSrv> sc_change_robot_mission_state_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetWorldOriginSrv>          sc_get_world_origin_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetSafetyBorderSrv>         sc_get_safety_border_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetObstaclesSrv>            sc_get_obstacles_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::GetMissionPointsSrv>        sc_get_mission_data_;
  mrs_lib::ServiceClientHandler<iroc_fleet_manager::srv::UploadFleetMissionSrv>      sc_upload_fleet_mission_;

  // | ----------------------- Action client ----------------------- |

  /** \brief Called when the fleet mission action completes. Resets goal handle and posts results to backend. */
  void missionDoneCallback(const rclcpp_action::ClientGoalHandle<Mission>::WrappedResult &result);

  /** \brief Called on fleet mission feedback. Broadcasts per-robot progress via the feedback WebSocket. */
  void missionFeedbackCallback(const Mission::Feedback::ConstSharedPtr feedback);

  // | ----------------------- Telemetry parsing ----------------------- |

  /** \brief Parses GeneralRobotInfo into JSON (battery, ready state, errors) and broadcasts via telemetry WebSocket. */
  void parseGeneralRobotInfo(mrs_msgs::msg::GeneralRobotInfo::ConstSharedPtr general_robot_info);

  /** \brief Parses StateEstimationInfo into JSON (pose, velocity, acceleration, estimators) and broadcasts. */
  void parseStateEstimationInfo(mrs_msgs::msg::StateEstimationInfo::ConstSharedPtr state_estimation_info, const std::string &robot_name);

  /** \brief Parses ControlInfo into JSON (controllers, trackers, thrust) and broadcasts. */
  void parseControlInfo(mrs_msgs::msg::ControlInfo::ConstSharedPtr control_info, const std::string &robot_name);

  /** \brief Parses CollisionAvoidanceInfo into JSON (enabled, avoiding, visible robots) and broadcasts. */
  void parseCollisionAvoidanceInfo(mrs_msgs::msg::CollisionAvoidanceInfo::ConstSharedPtr collision_avoidance_info, const std::string &robot_name);

  /** \brief Parses UavInfo into JSON (armed, offboard, flight state/duration, mass) and broadcasts. */
  void parseUavInfo(mrs_msgs::msg::UavInfo::ConstSharedPtr uav_info, const std::string &robot_name);

  /** \brief Parses SensorInfo (pre-formatted JSON details string) and broadcasts. */
  void parseSensorInfo(mrs_msgs::msg::SensorInfo::ConstSharedPtr sensor_info, const std::string &robot_name);

  /** \brief Parses SystemHealthInfo into JSON (CPU, RAM, HDD, rates, GNSS, sensors) and broadcasts. */
  void parseSystemHealthInfo(mrs_msgs::msg::SystemHealthInfo::ConstSharedPtr uav_info, const std::string &robot_name);

  // | ----------------------- JSON output methods ----------------------- |

  /** \brief Posts a JSON message to the backend HTTP server at /api/mission/{msg_type}. */
  void sendJsonMessage(const std::string &msg_type, json &json_msg);

  /** \brief Sends a JSON message via the mission feedback WebSocket channel. */
  void sendFeedbackJsonMessage(json &json_msg);

  /** \brief Adds a "type" field and sends a JSON message via the telemetry WebSocket channel. */
  void sendTelemetryJsonMessage(const std::string &type, json &json_msg);

  /**
   * \brief Finds a robot handler by name via linear search.
   * \return Pointer to the handler, or nullptr if not found.
   */
  robot_handler_t *findRobotHandler(const std::string &robot_name, robot_handlers_t &robot_handlers);

  // | ----------------------- Command dispatch ----------------------- |

  /**
   * \brief Dispatches a trigger command (takeoff/land/hover/home) to multiple robots.
   * Maps the command_type string to a service client member via trigger_command_handlers_.
   */
  action_result_t commandAction(const std::vector<std::string> &robot_names, const std::string &command_type);

  /**
   * \brief Generic template that applies a service call to multiple robots via member pointer.
   * Collects per-robot responses and returns aggregated success status with detailed message.
   */
  template <typename ServiceType>
  action_result_t commandAction(const std::vector<std::string> &robot_names, mrs_lib::ServiceClientHandler<ServiceType> robot_handler_t::*handler_member,
                                const std::shared_ptr<typename ServiceType::Request> &request);

  // | ----------------------- REST API callbacks ----------------------- |

  /** \brief POST: Sets the world origin (lat/lon) for all robots via set_origin service. */
  crow::response setOriginCallback(const crow::request &req);

  /** \brief GET: Returns the current world origin from the fleet manager. */
  crow::response getOriginCallback(const crow::request &req);

  /** \brief POST: Sets the safety border for all robots from a JSON polygon definition. */
  crow::response setSafetyBorderCallback(const crow::request &req);

  /** \brief GET: Returns the current safety border from the fleet manager. */
  crow::response getSafetyBorderCallback(const crow::request &req);

  /** \brief POST: Sets obstacles for all robots from a JSON obstacles array. */
  crow::response setObstacleCallback(const crow::request &req);

  /** \brief GET: Returns the current obstacles list from the fleet manager. */
  crow::response getObstaclesCallback(const crow::request &req);

  /** \brief POST: Uploads a fleet mission (type + details JSON) to the fleet manager. */
  crow::response uploadMissionCallback(const crow::request &req);

  /** \brief GET: Returns the current mission goal data from the fleet manager. */
  crow::response getMissionCallback(const crow::request &req);

  /**
   * \brief POST: Changes fleet-wide mission state (start/pause/stop).
   * For "start" with no active mission, sends an async action goal and waits for acceptance.
   */
  crow::response changeFleetMissionStateCallback(const crow::request &req, const std::string &type);

  /** \brief POST: Changes a single robot's mission state (start/pause/stop) by name. */
  crow::response changeRobotMissionStateCallback(const crow::request &req, const std::string &robot_name, const std::string &type);

  /** \brief GET: Returns a JSON array of available robots (name + type) that have sent at least one message. */
  crow::response availableRobotsCallback(const crow::request &req);

  /**
   * \brief POST: Dispatches a command to one robot (if specified) or all robots.
   * Routes to commandAction() with the appropriate robot name list.
   */
  crow::response commandCallback(const crow::request &req, const std::string &command_type, std::optional<std::string> robot_name);

  // | ----------------------- WebSocket callbacks ----------------------- |

  /**
   * \brief Handles WebSocket RC messages: "message" (echo) and "move" (velocity command).
   * Move commands scale x/y/z/heading by max_linear_speed_/max_heading_rate_ and call the velocity service.
   */
  void remoteControlCallback(crow::websocket::connection &conn, const std::string &data, bool is_binary);

  // | ----------------------- Service call helpers ----------------------- |

  /** \brief Convenience wrapper around iroc_common::callService() (fire-and-forget variant). */
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request);

  /** \brief Convenience wrapper around iroc_common::callService() (response-capturing variant). */
  template <typename ServiceType>
  result_t callService(mrs_lib::ServiceClientHandler<ServiceType> &sc, const std::shared_ptr<typename ServiceType::Request> &request,
                       const std::shared_ptr<typename ServiceType::Response> &response);

  // | ----------------------- Background threads & WebSocket state ----------------------- |

  std::thread th_death_check_;     ///< Monitors rclcpp::ok() and stops HTTP server on ROS shutdown.
  std::thread th_telemetry_check_; ///< Reserved for future telemetry monitoring.

  /** \brief Background thread that blocks until ROS shuts down, then gracefully stops the HTTP server. */
  void routine_death_check();

  crow::websocket::connection *active_telemetry_connection_ = nullptr; ///< Current telemetry WebSocket client (single-client model).
  std::mutex                   mtx_telemetry_connections_;             ///< Guards active_telemetry_connection_.
  crow::websocket::connection *active_feedback_connection_ = nullptr;  ///< Current feedback WebSocket client (single-client model).
  std::mutex                   mtx_feedback_connections_;              ///< Guards active_feedback_connection_.

  // | ----------------------- Mission action client ----------------------- |

  std::shared_ptr<MissionClient> mission_client_;      ///< Action client to the fleet manager's ExecuteMission action.
  MissionGoalHandle::SharedPtr   current_goal_handle_; ///< Currently active fleet mission goal handle.
  std::mutex                     mtx_current_goal_handle_;

  // | ----------------------- Cached state ----------------------- |

  mrs_msgs::msg::Point2D world_origin_; ///< Cached world origin (lat/lon), updated on setOriginCallback.
};

} // namespace iroc_bridge
