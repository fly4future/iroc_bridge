#!/usr/bin/env bash
curl -X POST -d '{"robot_name": "uav1", "frame_id": "local_origin", "points": [{"x": 0, "y": 0, "z": 10, "heading": 0}]}' "localhost:8080/set_path"
