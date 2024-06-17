#!/bin/bash

HELPER_PATH="~/workspace/.helper_scripts"


# Driver scripts
alias moveit_config_driver="$HELPER_PATH/driver-moveit-start.sh"
alias arm_drivers="$HELPER_PATH/drivers-arm-start.sh"
alias realsense_driver="$HELPER_PATH/driver-realsense-start.sh"
alias find_object_2d="$HELPER_PATH/find_object_2d.sh"

# Colcon helper scripts
alias build_workspace="source $HELPER_PATH/build-workspace.sh"