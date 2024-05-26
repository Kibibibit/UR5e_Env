#!/bin/bash

USER_UID="$(id -u)" \
    USER_GID="$(id -g)" \
    USERNAME="$(whoami)" \
    docker-compose down