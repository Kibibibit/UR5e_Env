#!/bin/bash


./.helper-scripts/get-user-confirmation.sh $@ \
    -m "Are you sure? This will delete your existing image and will potentially take a long time (About 5-10 minutes) to rebuild!"


if [ $? -eq 0 ]
then

    ./docker-kill.sh -y

    CONTAINER=`docker container ls --format '{{.Names}}' | grep ros2`
    IMAGE=`docker image ls --format '{{.Repository}}:{{.Tag}}' | grep ros:humble`

    if [ ! -z "$IMAGE" ] || [ ! -z "$CONTAINER" ]; then
        docker container rm ros2
        docker image rm ros:humble
    fi
fi