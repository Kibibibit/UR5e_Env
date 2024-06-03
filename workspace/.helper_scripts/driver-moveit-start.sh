#!/bin/bash

RVIZ=true
GRIPPER="-gripper"
PAR_ACTION_SERVER="moveit_action_server moveit_action_server.launch.py"

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
        --default-server )
            PAR_ACTION_SERVER="ur_moveit_config ur_moveit.launch.py"
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


ros2 launch $PAR_ACTION_SERVER ur_type:=ur5e launch_rviz:=$RVIZ \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/arm-with-camera$GRIPPER.urdf.xacro" \
    moveit_config_file:="/home/rosuser/workspace/src/par_pkg/srdf/arm-with-camera$GRIPPER.srdf.xacro"