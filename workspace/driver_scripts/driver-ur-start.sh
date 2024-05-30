#!/bin/bash


RVIZ=true
GRIPPER="-gripper"

while test $# != 0
do
    case "$1" in
        --no-rviz )
            RVIZ=false
            shift
            break
            ;;
        --no-gripper )
            GRIPPER=
            shift
            break
            ;;
        --) 
            shift
            break
            ;;
        * )
            echo "unrecognised argument $1!"
            exit 1
            break
            ;;
    esac
    shift
done



ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=10.234.6.49 \
    launch_rviz:=$RVIZ \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/arm-with-camera$GRIPPER.urdf.xacro" \
    moveit_config_file:="/home/rosuser/workspace/src/par_pkg/srdf/arm-with-camera$GRIPPER.srdf.xacro"