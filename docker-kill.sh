#!/bin/bash

./.helper-scripts/get-user-confirmation.sh $@ \
    -m "Are you sure? This stop and remove your existing container!"

if [ $? -eq 0 ]
then
    RUNNING=`docker ps --format '{{.Names}}' | grep ros2`
    if [ ! -z "$RUNNING" ]; then
        docker rm `docker kill ros2`
    fi

fi