#!/bin/bash

WIDTH=${1:-"110.0"}
FORCE=${2:-"40.0"}

ros2 action send_goal set_gripper_width par_interfaces/action/GripperSetWidth "{\"target_width\":$WIDTH, \"target_force\":$FORCE}"