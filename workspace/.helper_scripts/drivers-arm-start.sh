#!/bin/bash


RVIZ=true
GRIPPER=true
CAMERA=true

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
        --no-camera )
            CAMERA=false
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



ros2 launch par_pkg drivers.launch.py \
    robot_ip:=10.234.6.49 \
    ur_type:=ur5e launch_rviz:=$RVIZ low_poly:=true gripper:="$GRIPPER" \
    enable_camera:="$CAMERA" \
    description_file:="/home/rosuser/workspace/src/par_pkg/urdf/ur_assembly.urdf.xacro" \
    moveit_config_file:="/home/rosuser/workspace/src/par_pkg/srdf/ur_assembly.srdf.xacro"