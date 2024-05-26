#!/bin/bash

## This script creates the required display xsock and xauth files needed.
## This command may need to be run as sudo!

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH || echo "You may need to delete $XAUTH before trying to re-run this command"
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - || echo "You may need to delete $XAUTH before trying to re-run this command"