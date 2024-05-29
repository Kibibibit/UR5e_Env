#!/bin/bash

RVIZ=${1:-true}

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=10.234.6.49 launch_rviz:=$RVIZ