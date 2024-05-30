#!/bin/bash


SKIP_USER_CONFIRM=0
CONFIRMED=0
while test $# != 0
do
    case "$1" in
        -y) SKIP_USER_CONFIRM=1;;
        --) shift; break;;
    esac
    shift
done

if [[ "$SKIP_USER_CONFIRM" == "1" ]]; then
    CONFIRMED=1
else
    read -p "Are you sure? This will delete your existing image and take a long time (About 5-10 minutes) to rebuild! [y/n]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        CONFIRMED=1
    else
        exit 0
    fi
fi

if [[ "$CONFIRMED" == "1" ]]; then

    ./docker-kill.sh -y

    CONTAINER=`docker container ls --format '{{.Names}}' | grep ros2`
    IMAGE=`docker image ls --format '{{.Repository}}:{{.Tag}}' | grep ros:humble`

    if [ ! -z "$IMAGE" ] || [ ! -z "$CONTAINER" ]; then
        docker container rm ros2
        docker image rm ros:humble
    fi
fi