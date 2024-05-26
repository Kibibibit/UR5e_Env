#!/bin/bash

USER_UID="$(id -u)" \
    USER_GID="$(id -g)" \
    USERNAME=rosuser \
    docker-compose up -d