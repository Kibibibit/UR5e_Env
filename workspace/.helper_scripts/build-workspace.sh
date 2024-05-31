#!/bin/bash

# Allows building the package more easily without needing to remember which folder to be in

RETURN=`pwd`
WORKSPACE_FOLDER="/home/rosuser/workspace"
SRC_FOLDER="$WORKSPACE_FOLDER/src"

ALL_PKG=`ls $SRC_FOLDER`

PKG_LIST=$@


if [ -z "$PKG_LIST" ]
then
    PKG_LIST=$ALL_PKG
fi
echo -e "\033[0;92mBuilding:\033[0m\n\
$PKG_LIST"

cd $WORKSPACE_FOLDER
colcon build --packages-select $PKG_LIST
source $WORKSPACE_FOLDER/install/setup.bash
cd $RETURN
