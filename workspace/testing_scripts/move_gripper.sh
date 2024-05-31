#!/bin/bash

WIDTH=${1:-"110.0"}
FORCE=${2:-"40.0"}

ACTION="gripper_set_width par_interfaces/action/GripperSetWidth"

WIDTH_FIELD="\"target_width\":$WIDTH,"

if [ "$WIDTH" -eq "open" ]
then
    ACTION="gripper_full_open par_interfaces/action/GripperFullOpen"
    WIDTH_FIELD=
elif [ "$WIDTH" -eq "close" ]
then
    ACTION="gripper_full_close par_interfaces/action/GripperFullClose"
    WIDTH_FIELD=
fi

ros2 action send_goal $ACTION "{$WIDTH_FIELD\"target_force\":$FORCE}"