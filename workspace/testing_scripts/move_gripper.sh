#!/bin/bash

WIDTH=${1:-"110.0"}
FORCE=${2:-"40.0"}

ACTION="rg2/set_width onrobot_rg2_msgs/action/GripperSetWidth"

WIDTH_FIELD="\"target_width\":$WIDTH,"

if [ "$WIDTH" == "open" ]
then
    ACTION="rg2/full_open onrobot_rg2_msgs/action/GripperFullOpen"
    WIDTH_FIELD=
elif [ "$WIDTH" == "close" ]
then
    ACTION="rg2/full_close onrobot_rg2_msgs/action/GripperFullClose"
    WIDTH_FIELD=
fi

ros2 action send_goal $ACTION "{$WIDTH_FIELD\"target_force\":$FORCE}"