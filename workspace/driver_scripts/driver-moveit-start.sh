#!/bin/bash

RVIZ=${1:-true}
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=$RVIZ