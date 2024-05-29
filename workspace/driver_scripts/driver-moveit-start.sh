#!/bin/bash

RVIZ=${1:-true}
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=$RVIZ \
    description_file:="$(find par_pkg)/home/rosuser/workspace/src/par_pkg/urdf/arm-with-joint.xacro"