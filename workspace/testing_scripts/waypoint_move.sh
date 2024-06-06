#!/bin/bash

X=$1
Y=$2
Z=$3
R=$4

ros2 action send_goal \
    par_moveit/waypoint_move par_interfaces/action/WaypointMove \
    "{\"target_pose\":{\"position\":{\"x\":$X, \"y\":$Y,\"z\":$Z}, \"rotation\":$R}}"