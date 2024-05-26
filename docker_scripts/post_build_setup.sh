#!/bin/bash


MIXINS=`colcon mixin list`
WORKSPACE=~/workspace/src
if [ -z "$MIXINS" ]; then
    echo "No mixins added, adding now..."
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default
    cd $WORKSPACE
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    cd ..
    rosdep update && rosdep install --from-paths src --ignore-src -y
    cd ~
fi
