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
    read -p "Are you sure? This stop and remove your existing container! [y/n]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        CONFIRMED=1
    else
        exit 0
    fi
fi

if [[ "$CONFIRMED" == "1" ]]; then

    RUNNING=`docker ps --format '{{.Names}}' | grep ros2`
    if [ ! -z "$RUNNING" ]; then
        docker rm `docker kill ros2`
    fi

fi