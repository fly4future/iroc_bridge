#!/usr/bin/env bash
curl -X POST -d '{"robot_name": "uav1", "type": "pause"}' "localhost:8080/set_path"

