#!/bin/bash

# Allows building the package more easily without needing to remember which folder to be in

# Get the users' current folder to return to
RETURN=`pwd`
# The actual workspace to build
WORKSPACE_FOLDER="/home/rosuser/workspace"
# The file folder containing all packages
SRC_FOLDER="$WORKSPACE_FOLDER/src"

# Get a list of all packages
ALL_PKG=`ls $SRC_FOLDER`

# We want all parameters to see if the user has specified any specific packages to build
PKG_LIST=$@


# If they haven't, we use the list of all packages
if [ -z "$PKG_LIST" ]
then
    PKG_LIST=$ALL_PKG
fi
echo -e "\033[0;92mBuilding:\033[0m\n\
$PKG_LIST"

# Build, source and return home
cd $WORKSPACE_FOLDER
colcon build --packages-select $PKG_LIST
source $WORKSPACE_FOLDER/install/setup.bash
cd $RETURN
