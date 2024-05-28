#!/bin/bash

# Sources everything before running


./docker-attach.sh /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /home/${USERNAME}/workspace/install/setup.bash && \
    source /home/${USERNAME}/.moveit_workspace/install/setup.bash && \
    $1" 