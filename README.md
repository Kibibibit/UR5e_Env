# UR5e_Env

A Ros2 environment for working with Moveit2 on the ur5e arm.

## Scripts

|Name|Function|
|-|-|
|[initial_setup.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/initial_setup.sh)| This script sets up the required xsock and xauth files for the docker containers to be able to run Rviz. Should be run first before anything else. |
|[docker-build.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-build.sh)| Builds the docker container. It also completely deletes the previous image when run, so be aware of that. It will not delete anything you've placed in `/workspace/src` however, as that is stored in a volume. Adding the `-y` flag will skip asking for confirmation.|
|[docker-start.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-start.sh)| Runs the docker container in headless mode. You can use `docker-attach.sh` to access it. |
|[docker-stop.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-stop.sh)| Stops the docker container without destroying it, unlike `docker-kill.sh` |
|[docker-attach.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-attach.sh)| Provides terminal access to the docker container. You can exit with `CTRL+D` without destroying the container. |
|[docker-kill.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-kill.sh)| Kills and removes the container. Adding the `-y` flag will skip asking for confirmation.|
|[docker-delete.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-delete.sh)| Deletes the ROS docker image from the system. This is needed to clear some errors that can occur when rebuilding the container, and is run automatically by `docker-build.sh`. The `-y` flag can skip asking for confirmation.|

## Installation
### Requirements
- docker
- docker-compose

### Installing moveit
Moveit has to be built from source manually after starting the docker. Run the following steps:
First run this to create the required files for RVIZ to connect to the docker.
```bash
./initial_setup.sh
``` 
Then, run the following to build, start and connect to the docker.
```bash
./docker-build.sh
./docker-start.sh
./docker-attach.sh
```
Finally, run the following to start the moveit install.
```bash
cd ~/rosuser/.post-build
./moveit_install.sh
```
You'll need to disconnect `CTRL+D` and reconnect with `docker-attach.sh` to source the new moveit repo. 


## Usage

### Starting Up
1. In the the `UR5e_Env` folder, first run `./initial_setup.sh`. This script creates the necassary files for Rviz to be able to access your display outside of the docker container. Without this script, everything will still work, but RVIZ will not render.
2. Run the script `./docker-build.sh`. This builds the docker container, and will take 5+ minutes to run on the first build. Future builds will be signifigantly faster.
3. Run the script `./docker-start.sh`. This will start the docker container, and you are now ready to start working.

### Attaching to the environment
#### Bash
To access the docker container, to run scripts, you can use `./docker-attach.sh`. This will open a terminal inside the container for running commands.
#### VSCode
You can also access the workspace inside the docker container with the vscode exension [dev containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). Once the container is running, hit `ctrl+shift+P`, and type `attach`, and find the option: `Dev Containers: Attach to running container`. Select the container, and when asked what folder to use, select `/home/rosuser/workspace/`. You can now develop inside the container. All your changes will be saved into the workspace folder on your local machine.

### Running UR5e Drivers
#### UR Controller
1. First, on the UR5e teach pendant, select: `Open>>Installation>>RemoteRos`. If asked to update the program, select `Update Program`.
2. Then, go to the `Program` tab, and under `URCaps`, add `External Control`. Make sure the IP matches the PC IP, and that the port is 50002.
3. Turn ON the arm, ensuring that the E-Stop is released.
4. Attach to the container (`./docker-attach.sh`) and run `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=10.234.6.49 launch_rviz:=true`. You can omit Rviz if you'd like. *TODO: Using a different description (xacro) file.*
5. An RViz display showing the current state of the robot should appear.
#### MoveIt
1. in a new terminal tab, attach to the container (`./docker-attach.sh`)
2. Run `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true` *TODO: Using a different description (xacro) file.*
3. You can now move the blue sphere to move the arm around.
4. Click `plan` to visualise the trajectory of the movement, and then `execute` to move the arm.

### Creating new Packages
*TODO* (Basically just put them in workspace/src but I'll write something proper soon)

### Shutting down
#### Stopping the container
`./docker-stop.sh` will shut down the container, but leave it for future use. It can be restarted with `./docker-start.sh`
#### Destroying the container
If you need to rebuild the container, you should run `./docker-delete.sh`, which will wipe the container and require a rebuild to use again. You shouldn't have to do this very often.


## Credits
*TODO Format nicely*
Sam/Jasper
Some ros wiki pages, will find and add when I get a chance.