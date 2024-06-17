#!/bin/bash

X=$1
Y=$2
R=${3:-"0"}



STRING=`ros2 service call par/board_to_world par_interfaces/srv/BoardToWorld "{\"board_pos\":{\"x\":$X, \"y\":$Y}}"`


X_REGEX="x=[0-9\.]*"
Y_REGEX="y=[0-9\.]*"
Z_REGEX="z=[0-9\.]*"


X_PREFIX="x="
Y_PREFIX="y="
Z_PREFIX="z="

[[ "$STRING" =~ $X_REGEX ]]
X_P=${BASH_REMATCH#"$X_PREFIX"}
[[ "$STRING" =~ $Y_REGEX ]]
Y_P=${BASH_REMATCH#"$Y_PREFIX"}
[[ "$STRING" =~ $Z_REGEX ]]
Z_P=${BASH_REMATCH#"$Z_PREFIX"}

    
# SET Z to 0 till we can trust table height
Z_P=0

./waypoint_move.sh $X_P $Y_P $Z_P $R

