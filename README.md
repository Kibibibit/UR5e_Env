# UR5e_Env

A Ros2 environment for working with Moveit! on the ur5e arm.

## Scripts

|Name|Function|
|-|-|
|[initial_setup.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/initial_setup.sh)| This script sets up the required xsock and xauth files for the docker containers to be able to run Rviz. Should be run first before anything else. |
|[docker-build.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-build.sh)| Builds the docker container. It also completely deletes the previous image when run, so be aware of that. It will not delete anything you've placed in `/workspace/src` however, as that is stored in a volume. Adding the `-y` flag will skip asking for confirmation.|
|[docker-start.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-start.sh)| Runs the docker container in headless mode. You can use `docker-attach.sh` to access it. |
|[docker-stop.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-stop.sh)| Stops the docker container without destroying it, unlike `docker-kill.sh` |
|[docker-attach.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-attach.sh)| Provides terminal access to the docker container. You can exit with `CTRL+D` without destroying the container. |
|[docker-kill.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-kill.sh)| Kills and removes the container. Adding the `-y` flag will skip asking for confirmation.|
|[docker-delete.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-delete.sh)| Deletes the ROS docker image from the system. This is needed to clear some errors that can occur when rebuilding the container, and is run automatically by `docker-delete.sh`. The `-y` flag can skip asking for confirmation.|

## Initial Setup
TODO
