#!/bin/bash

COMMAND=${1:-/bin/bash}

docker exec -w /home/rosuser/workspace --user rosuser -it ros2 $COMMAND