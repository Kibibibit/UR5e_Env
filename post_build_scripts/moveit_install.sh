#!/bin/bash

WORKSPACE=$HOME/.moveit_workspace
MARKER_FILE=$WORKSPACE/.moveit_install_done

if [ ! -f "$MARKER_FILE" ]; then
    mkdir -p $WORKSPACE/src
    cd $WORKSPACE/src
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default
    rosdep update
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/moveit2_tutorials
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd $WORKSPACE
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    touch $MARKER_FILE
    
fi

source $WORKSPACE/install/setup.bash
