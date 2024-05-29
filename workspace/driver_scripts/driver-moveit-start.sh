#!/bin/bash
#Need to set up proper parameters for this   
RVIZ=${1:-true}
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=$RVIZ \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/arm-with-gripper.urdf.xacro" \
    moveit_config_file:='/home/rosuser/workspace/src/par_pkg/srdf/arm-with-gripper.srdf.xacro'