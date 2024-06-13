#!/bin/bash

RVIZ=true
GRIPPER=true
PAR_ACTION_SERVER="par_moveit_config par_moveit_config.launch.py"

while test $# != 0
do
    case "$1" in
        --no-rviz )
            RVIZ=false
            shift
            break
            ;;
        --no-gripper )
            GRIPPER=false
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


ros2 launch $PAR_ACTION_SERVER ur_type:=ur5e launch_rviz:=$RVIZ low_poly:=true gripper:=$GRIPPER \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/ur_assembly.urdf.xacro" \
    moveit_config_package:=par_pkg \
    moveit_config_file:="/home/rosuser/workspace/src/par_pkg/srdf/ur_assembly.srdf.xacro"