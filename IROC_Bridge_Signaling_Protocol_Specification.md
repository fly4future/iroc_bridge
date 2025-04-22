## IROC Bridge Signaling Protocol Specification
This documents outlines the signaling protocol used by the IROC Bridge package to communicate between clients.
Designed to communicate between web client and ROS.

## HTTP API

You can use the HTTP API to send requests to interact with the robots and receive status information.
The requests and responses are in JSON format.

### Requests
#### Status
* <span style="color:green"> 
 `GET` 
 </span> 
 **Avalaible Robots** \
`http://localhost:8080/available_robots`

#### Actions
* <span style="color:orange"> 
 `POST` 
 </span> 
 **Takeoff** \
`http://localhost:8080/takeoff`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "robot_names": [
      "uav1"
    ]
  }
  ```
</details>

* <span style="color:green"> 
 `GET` </span> 
 **Takeoff All** \
`http://localhost:8080/takeoff_all`

* <span style="color:orange"> 
 `POST` </span> 
 **Hover** \
`http://localhost:8080/hover`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "robot_names": [
      "uav1"
    ]
  }
  ```
</details>

* <span style="color:green"> 
 `GET` </span> 
 **Hover All** \
`http://localhost:8080/hover_all`

* <span style="color:orange"> 
 `POST` 
 </span> 
 **Land** \
`http://localhost:8080/land`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "robot_names": [
      "uav1"
    ]
  }
  ```
</details>

* <span style="color:green"> `GET` </span> **Land All** \
`http://localhost:8080/land_all`

* <span style="color:orange"> 
 `POST` 
 </span> 
 **Land Home** \
`http://localhost:8080/land_home`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "robot_names": [
      "uav1"
    ]
  }
  ```
</details>

* <span style="color:green"> 
  `GET` </span> 
  **Land Home All** \
`http://localhost:8080/land_home_all`

#### Safety area configuration

* <span style="color:orange"> 
  `POST` </span> 
  **Set Safety Border** \
`http://localhost:8080/set_safety_border`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
      "points": [
          {
              "x": 47.397760,
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

* <span style="color:orange"> 
  `POST` 
  </span> 
  **Set Obstacle** \
`http://localhost:8080/set_obstacle`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "points": [
        {
            "x": 47.397760,
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

#### Mission commands

* <span style="color:orange"> 
  `POST` 
  </span> 
  **Set Waypoint Mission** \
`http://localhost:8080/set_waypoint_mission`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>
  ```json
  {
      "mission": [
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
          },
          {
              "robot_name": "uav2",
              "frame_id": 0,
              "height_id": 0,
              "points": [
                  {
                      "x": 20,
                      "y": 5,
                      "z": 3,
                      "heading": 0
                  }
              ],
              "terminal_action": 0
          }
      ]
  }
  ```
</details>

* <span style="color:orange"> 
 `POST` 
 </span> 
 **Set Autonomy Test** \
`http://localhost:8080/set_autonomy_test`
<details>
  <summary>
  **Body** 
  <span style="color:gray"> raw (json) </span>
  </summary>

  ```json
  {
    "robot_name": "uav1",
    "segment_length": 2
  }
  ```
</details>

## WebSocket API

You can use the WebSocket API to receive telemetry and send requests to control the robots. 

### Telemetry

Robots data and status can be received periodically in the following path: \
`ws://localhost:8080/telemetry`

* <span style="color:green"> 
 `onmessage` 
 </span> 
 **General Robot Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
\
* <span style="color:green"> 
  `onmessage` 
  </span> 
  **State Estimation Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
\  
* <span style="color:green"> 
  `onmessage` 
  </span> 
  **Control Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
\
* <span style="color:green"> 
  `onmessage` 
  </span> 
  **Collision Avoidance  Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
\
* <span style="color:green">
  `onmessage` 
  </span> 
  **UAV Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
\
* <span style="color:green">
  `onmessage` </span>
  **System Health Info** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
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
      [
        "/uav2/hw_api",
        1.09215
      ],
      [
        "/uav2/constraint_manager",
        1.09215
      ],
      [
        "/uav2/control_manager",
        1.09215
      ],
      [
        "/uav2/estimation_manager",
        0
      ]
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
  
  
#### Missions Feedback  

* <span style="color:green">
 `onmessage`
 </span>
 **Waypoint Mission and Autonomy Test Feedback** \
<details>
  <summary>
  **Message** <span style="color:gray"> raw (json) </span>
  </summary>
  
  ```json
  {
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
> Autonomy test follows the same structure as the waypoint mission feedback, however it will always contain only one robot. 


 
