# IROC Bridge Signaling Protocol Specification

This document outlines the signaling protocol IROC Bridge package uses to communicate with clients.
Designed to communicate between the web client and ROS.

## HTTP API

You can use the HTTP API to send requests to interact with the robots and receive status information.
The requests and responses are in JSON format.

### Robot control

Endpoints for controlling the robots.

- <b style="color: #61affe">`GET`</b>
  **/robots**  
  <span style="color: gray">
  List available robots.
  </span>
- <b style="color: #48cc90">`POST`</b>
  **/robots/{_robot_name_}/takeoff**  
  <span style="color: gray">
  Command takeoff (single)
  </span>
- <b style="color: #49CC90">`POST`</b>
  **/robots/takeoff**  
  <span style="color: gray">
  Takeoff all robots
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/hover**  
  <span style="color: gray">
  Command hover (single)
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/hover**  
  <span style="color: gray">
  Hover all robots
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/land**  
  <span style="color: gray">
  Command land (single)
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/land**  
  <span style="color: gray">
  Land all robots
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/home**  
  <span style="color: gray">
  Command land home (single)
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/home**  
  <span style="color: gray">
  Land home all robots
  </span>

### Environment setup

Endpoints for controlling the robot environment.

- <b style="color: #49cc90">`POST`</b>
  **/safety-area/borders**  
  <span style="color: gray">
  Set the safety area borders.
  </span>
  <details>
  <summary><i>Body</i> <span style="color: gray">raw (json)</span></summary>

  ```json
  {
    "points": [
      {
        "x": 47.39776,
        "y": 8.545254
      },
      {
        "x": 47.397719,
        "y": 8.545436
      },
      {
        "x": 47.397601,
        "y": 8.545367
      },
      {
        "x": 47.397657,
        "y": 8.545191
      }
    ],
    "height_id": 1,
    "max_z": 347,
    "min_z": 343
  }
  ```

  </details>

- <b style="color: #49cc90">`POST`</b>
  **/safety-area/obstacles**  
  <span style="color: gray">
  Set the safety area obstacles.
  </span>
  <details>
  <summary>
  <i>Body</i> <span style="color: gray">raw (json)</span>
  </summary>

  ```json
  {
    "points": [
      {
        "x": 47.39776,
        "y": 8.545254
      },
      {
        "x": 47.397719,
        "y": 8.545436
      },
      {
        "x": 47.397601,
        "y": 8.545367
      },
      {
        "x": 47.397657,
        "y": 8.545191
      }
    ],
    "height_id": 1,
    "max_z": 347,
    "min_z": 343
  }
  ```

  </details>

### Missions

The missions are handled by `IROC Fleet Manager`: node responsible of sending the mission to the robots, monitoring their progress and sending the aggregated information to the `IROC Bridge`.

- <b style="color: #49cc90">`POST`</b>
  **/mission/waypoints**  
  <span style="color: gray">
  Set the waypoints for the mission.
  </span>
  <details>
  <summary>
  <i>Body</i> <span style="color: gray">raw (json)</span>
  </summary>

  ```json
  {
    "robot_name": "uav1",
    "frame_id": 0,
    "height_id": 0,
    "points": [
      {
        "x": 10,
        "y": 10,
        "z": 2,
        "heading": 1
      },
      {
        "x": -10,
        "y": 10,
        "z": 2,
        "heading": 3
      }
    ],
    "terminal_action": 1
  }
  ```

  </details>

- <b style="color: #49cc90">`POST`</b>
  **/mission/autonomy-test**  
  <span style="color: gray">
  Set the autonomy test for the mission.
  </span>
  <details>
  <summary>
  <i>Body</i> <span style="color: gray">raw (json)</span>
  </summary>

  ```json
  {
    "robot_name": "uav1",
    "frame_id": 0,
    "height_id": 0,
    "points": [
      {
        "x": 10,
        "y": 10,
        "z": 2,
        "heading": 1
      },
      {
        "x": -10,
        "y": 10,
        "z": 2,
        "heading": 3
      }
    ],
    "terminal_action": 1
  }
  ```
  </details>

#### Feedback

During an active mission, the feedback message is broadcasted to the connected clients through a WebSocket in the `/telemetry` path.

- <b style="color: orange">`onmessage`</b>
  **Waypoint Mission and Autonomy Test Feedback.**
  <details>
  <summary>
  *Message* <span style="color: gray">raw (json)</span>
  </summary>

  ```json
  {
    "type": "WaypointMissionFeedback",
    "progress": 0.75,
    "mission_state": "IN_PROGRESS",
    "message": "EXECUTING",
    "robots": [
      {
        "robot_name": "uav1",
        "message": "EXECUTING",
        "mission_progress": 0.6,
        "current_goal": 2,
        "distance_to_goal": 15.3,
        "goal_estimated_arrival_time": 30,
        "goal_progress": 0.8,
        "distance_to_finish": 50.2,
        "finish_estimated_arrival_time": 50
      },
      {
        "robot_name": "uav2",
        "message": "EXECUTING",
        "mission_progress": 0.45,
        "current_goal": 1,
        "distance_to_goal": 5.7,
        "goal_estimated_arrival_time": 30,
        "goal_progress": 0.95,
        "distance_to_finish": 75.8,
        "finish_estimated_arrival_time": 50
      }
    ]
  }
  ```

  </details>

> [!NOTE]
> Autonomy test follows the same structure as the waypoint mission feedback, but it will always contain only one robot.

#### Result

When a mission is finished, the result message message will be sent to

<b style="color: #49cc90">`POST`</b>
**http://server:8000/api/missions/result**  
<span style="color: gray">
Send the result of the mission.
</span>

<details>
  <summary>
  <i>Body</i> <span style="color: gray">raw (json)</span>
  </summary>

```json
{
  "success": true,
  "message": "All robots finished succesfully",
  "robot_results": [
    {
      "robot_name": "uav1",
      "success": true,
      "message": "Robot finished successfully"
    },
    {
      "robot_name": "uav2",
      "success": true,
      "message": "Robot finished successfully"
    }
  ]
}
```

</details>

#### Mission Control Endpoints

We support for both fleet-wide and individual robot mission control.

##### Fleet Mission Control:

These endpoints control the mission status for all assigned robots at once: \

- <b style="color: #49cc90">`POST`</b>
  **/mission/start**  
  <span style="color: gray">
  Start the mission for all robots.
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/mission/pause**  
  <span style="color: gray">
  Pause the mission for all robots.
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/mission/stop**  
  <span style="color: gray">
  Stop the mission for all robots.
  </span>

##### Robot Mission Control:

You can also control individual mission robots using these endpoints:

- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/mission/start**  
   <span style="color: gray">
  Start the mission for a specific robot.
  </span>

  > [!NOTE]
  > Starting a mission for a single robot will activate that robot while the others remain in a waiting state. You can later use the `/mission/start` endpoint to activate the remaining robots and continue the mission.

- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/mission/pause**  
  <span style="color: gray">
  Pause the mission for a specific robot.
  </span>
- <b style="color: #49cc90">`POST`</b>
  **/robots/{_robot_name_}/mission/stop**  
   <span style="color: gray">
  Stop the mission for a specific robot.
  </span>
  > [!NOTE]
  > Stopping the mission for a single robot will also abort the overall mission and stop all other robots. This behavior is intentional, as the mission assumes the participation of all assigned robots.

## WebSocket API

You can use the WebSocket API to receive robots telemetry and send requests to control the robots.

### Telemetry

Robot's data and status can be received periodically in the `/telemetry` path.

- <b style="color: orange">`onmessage`</b>
  **General Robot Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>

  ```json
  {
    "errors": [],
    "type": "GeneralRobotInfo",
    "ready_to_start": 1,
    "problems_preventing_start": [],
    "battery_state": {
      "wh_drained": -1,
      "percentage": -1,
      "voltage": -1
    },
    "robot_type": 0,
    "robot_name": "uav2"
  }
  ```

  </details>

- <b style="color: orange">`onmessage`</b>
  **State Estimation Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>
    
    ```json
    {
      "type": "StateEstimationInfo",
      "switchable_estimators": [
        "gps_baro",
        "gps_garmin"
      ],
      "velocity": {
        "angular": {
          "z": 0,
          "y": 0,
          "x": 0
        },
        "linear": {
          "z": 4.6765261112091244e-21,
          "y": 0,
          "x": 0
        }
      },
      "global_pose": {
        "heading": 1.02729905983773,
        "altitude": 340,
        "longitude": 8.545800727209587,
        "latitude": 47.39776586900617
      },
      "local_pose": {
        "z": 0.059999996605801006,
        "heading": 1.02729905983773,
        "y": 2.4504742256806935,
        "x": 15.614331170562465
      },
      "current_estimator": "gps_baro",
      "above_ground_level_height": 0.059999996605801,
      "running_estimators": [
        "gps_baro",
        "gps_garmin"
      ],
      "acceleration": {
        "angular": {
          "z": 0,
          "y": 0,
          "x": 0
        },
        "linear": {
          "z": 1.0095692646347513e-18,
          "y": 0,
          "x": 0
        }
      },
      "estimation_frame": "uav2/gps_garmin_origin",
      "robot_name": "uav2"
    }
    ```
  </details>

- <b style="color: orange">`onmessage`</b>
  **Control Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>
    
    ```json
    {
      "type": "ControlInfo",
      "thrust": null,
      "available_trackers": [],
      "active_tracker": "unknown",
      "available_controllers": [],
      "active_controller": "unknown",
      "robot_name": "uav2"
    }
    ```
  </details>

- <b style="color: orange">`onmessage`</b>
  **Collision Avoidance Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>
    
    ```json
    {
      "type": "CollisionAvoidanceInfo",
      "other_robots_visible": [
        "uav1"
      ],
      "collision_avoidance_enabled": 1,
      "avoiding_collision": 0,
      "robot_name": "uav2"
    }
    ```
  </details>

- <b style="color: orange">`onmessage`</b>
  **UAV Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>

  ```json
  {
    "mass_nominal": null,
    "type": "UavInfo",
    "flight_duration": 0,
    "flight_state": "OFFBOARD",
    "offboard": 1,
    "armed": 1,
    "robot_name": "uav2"
  }
  ```

  </details>

- <b style="color: orange">`onmessage`</b>
  **System Health Info**
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>

  ```json
  {
    "free_ram": 22.789223,
    "robot_name": "uav2",
    "cpu_load": 10.102389,
    "mag_strength": null,
    "total_ram": 30.061069,
    "type": "SystemHealthInfo",
    "mag_uncertainty": null,
    "free_hdd": 1393,
    "state_estimation_rate": 20.080807,
    "hw_api_rate": 99.019608,
    "control_manager_rate": 0.990196,
    "gnss_uncertainty": 0,
    "node_cpu_loads": [
      ["/uav2/hw_api", 1.09215],
      ["/uav2/constraint_manager", 1.09215],
      ["/uav2/control_manager", 1.09215],
      ["/uav2/estimation_manager", 0]
    ],
    "available_sensors": [
      {
        "name": "pixhawk",
        "status": "NOT_IMPLEMENTED",
        "ready": 1,
        "rate": -1
      },
      {
        "rate": -1,
        "ready": 1,
        "status": "NOT_IMPLEMENTED",
        "name": "garmin_down"
      }
    ]
  }
  ```

  </details>

### Robot remote control

You can use the WebSocket API to control the robots in the `/rc` path.

- <b style="color: orange">`onmessage`</b>
  **Message**  
  <span style="color: gray">
  Similar to a ping websocket message.
  </span>
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>

  ```json
  {
    "command": "message",
    "data": "Hello, World!"
  }
  ```

  </details>

- <b style="color: orange">`onmessage`</b>
  **Movement**  
  <span style="color: gray">
  To control the UAV, it receives normalized linear (`x`, `y`, `z`) and angular (`yaw`) velocities.
  </span>
  <details>
    <summary>
    *Message* <span style="color: gray">raw (json)</span>
    </summary>

  ```json
  {
    "command": "move",
    "robot_name": "uav1",
    "data": {
      "x": 1.0,
      "y": -0.5,
      "z": 0,
      "heading": 1.0
    }
  }
  ```

  </details>

## Camera stream using WebRTC

The features for the camera streaming are available, and the setup can be tested by starting the simulator with the camera argument for that will start the gazebo simulator:

```sh
./start --camera
```

This will start the WebRTC server and allow the camera stream to be visualized on port `9090` of the server.

> [!NOTE]
> Please follow the instructions for the installation of dependencies in the [webrtc_ros](https://github.com/fly4future/webrtc_ros) repository. A detailed example of how the integration can be done is [here](https://github.com/fly4future/webrtc_ros/blob/develop/web/TUTORIAL.md).
