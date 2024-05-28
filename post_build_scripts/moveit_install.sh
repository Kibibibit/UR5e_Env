#!/bin/bash

WORKSPACE=$HOME/workspace/src/
MOVEIT_FOLDER=$WORKSPACE/moveit
MARKER_FILE=$MOVEIT_FOLDER/.moveit_install_done

if [ ! -f "$MARKER_FILE" ]; then
    mkdir -p $MOVEIT_FOLDER

    cd $MOVEIT_FOLDER
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/moveit2_tutorials
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    source /opt/ros/$ROS_DISTRO/setup.bash
    rosdep update
    cd $WORKSPACE
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    source $WORKSPACE/install/setup.bash

    touch $MARKER_FILE
    
fi
