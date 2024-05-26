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
    read -p "Are you sure? This will delete your exising container and image, and take potentially a long time (500+ seconds!) to rebuild! [y/n]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        CONFIRMED=1
    else
        exit 0
    fi
fi

if [[ "$CONFIRMED" == "1" ]]; then
    ./docker-delete.sh -y
    USER_UID="$(id -u)" \
        USER_GID="$(id -g)" \
        USERNAME=rosuser \
        docker-compose build
    
fi



