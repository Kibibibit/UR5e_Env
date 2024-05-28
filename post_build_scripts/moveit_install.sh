#!/bin/bash

MOVEIT_WORKSPACE=$HOME/.moveit_workspace

mkdir -p $MOVEIT_WORKSPACE/src
cd $MOVEIT_WORKSPACE
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
rosdep update
cd $MOVEIT_WORKSPACE/src
git clone --branch $ROS_DISTRO https://github.com/ros-planning/moveit2_tutorials
vcs import < moveit2_tutorials/moveit2_tutorials.repos
source /opt/ros/$ROS_DISTRO/setup.bash
cd $MOVEIT_WORKSPACE
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release

source $MOVEIT_WORKSPACE/install/setup.bash
