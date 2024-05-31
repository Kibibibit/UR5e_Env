#!/bin/bash

MESSAGE=

SKIP_USER_CONFIRM=0
CONFIRMED=0
while test $# != 0
do
    case "$1" in
        -m)
            MESSAGE=$2
            shift 1
            break
            ;;
        -y) 
            SKIP_USER_CONFIRM=1
            ;;
        --) 
            shift
            break
            ;;
    esac
    shift
done

if [[ "$SKIP_USER_CONFIRM" == "1" ]]
then
    exit 0
else
    echo -e "\033[1;31m$MESSAGE [y/n]:\033[0m "
    read -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        exit 0
    else
        exit 1
    fi
fi