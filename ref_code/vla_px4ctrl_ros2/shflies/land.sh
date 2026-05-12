#!/usr/bin/env bash
set -e

ros2 topic pub --once /drone6/px4ctrl/takeoff_land quadrotor_msgs/msg/TakeoffLand "{takeoff_land_cmd: 2}"
