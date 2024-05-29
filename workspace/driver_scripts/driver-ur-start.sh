#!/bin/bash

RVIZ=${1:-true}

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=10.234.6.49 \
    launch_rviz:=$RVIZ \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/arm-with-gripper.urdf.xacro" \
    moveit_config_file:='/home/rosuser/workspace/src/par_pkg/srdf/arm-with-gripper.srdf.xacro'