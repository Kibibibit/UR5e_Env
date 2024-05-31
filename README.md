# UR5e_Env

A Ros2 environment for working with Moveit2 on the ur5e arm.

## Scripts

|Name|Function|
|-|-|
|[docker-build.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-build.sh)| Builds the docker container. It also completely deletes the previous image when run, so be aware of that. It will not delete anything you've placed in `/workspace/src` however, as that is stored in a volume. Adding the `-y` flag will skip asking for confirmation.|
|[docker-start.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-start.sh)| Runs the docker container in headless mode. You can use `docker-attach.sh` to access it. |
|[docker-stop.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-stop.sh)| Stops the docker container without destroying it, unlike `docker-kill.sh` |
|[docker-attach.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-attach.sh)| Provides terminal access to the docker container. You can exit with `CTRL+D` without destroying the container. |
|[docker-kill.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-kill.sh)| Kills and removes the container. Adding the `-y` flag will skip asking for confirmation.|
|[docker-delete.sh](https://github.com/Kibibibit/UR5e_Env/blob/main/docker-delete.sh)| Deletes the ROS docker image from the system. This is needed to clear some errors that can occur when rebuilding the container, and is run automatically by `docker-build.sh`. The `-y` flag can skip asking for confirmation.|

## Installation
### Requirements
- [Docker](https://docs.docker.com/engine/install/ubuntu/) - Don't use apt without following this guide!
- [Docker Post-install](https://docs.docker.com/engine/install/linux-postinstall/) - If you don't follow this, it won't work!
- `docker-compose` - sudo apt install docker-compose

### Installing moveit
Moveit has to be built from source manually after starting the docker. Run the following steps:
```bash
./docker-build.sh
./docker-start.sh
./docker-attach.sh
```
Finally, run the following to start the moveit install. This takes about 30 minutes, but should only need to be run once.
```bash
cd ~/rosuser/.post-build
./moveit_install.sh
```
You'll need to disconnect `CTRL+D` and reconnect with `docker-attach.sh` to source the new moveit repo. 


## Usage

### Starting Up
1. Run the script `./docker-build.sh`. This builds the docker container, and will take 5+ minutes to run on the first build. Future builds will be signifigantly faster.
2. Run the script `./docker-start.sh`. This will start the docker container, and you are now ready to start working.

### Attaching to the environment
#### Bash
To access the docker container, to run scripts, you can use `./docker-attach.sh`. This will open a terminal inside the container for running commands.
#### VSCode
You can also access the workspace inside the docker container with the vscode exension [dev containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). Once the container is running, hit `ctrl+shift+P`, and type `attach`, and find the option: `Dev Containers: Attach to running container`. Select the container, and when asked what folder to use, select `/home/rosuser/workspace/`. You can now develop inside the container. All your changes will be saved into the workspace folder on your local machine.

### Running UR5e Drivers
#### UR Controller
First, on the UR5e teach pendant, select: `Open>>Installation>>RemoteRos`. If asked to update the program, select `Update Program`.<br/>
Then, go to the `Program` tab, and under `URCaps`, add `External Control`. Make sure the IP matches the PC IP, and that the port is 50002.<br/>
Turn ON the arm, ensuring that the E-Stop is released.<br/>
Attach to the container (`./docker-attach.sh`) and run:
```sh
ur_driver
```
An RViz display showing the current state of the robot should appear.<br/>
If you want to disable RVIZ, you can add `--no-rviz` to the end of the command. If you're working without the gripper, you can add `--no-gripper` to disable it.<br/> 
#### MoveIt
In a new terminal tab, attach to the container (`./docker-attach.sh`) <br/>
Run:
```sh
moveit_config_driver
```
You can now move the blue sphere to move the arm around, and use `plan` to visualise the trajectory of the movement, and then `execute` to move the arm. <br/>
This is needed for moveit commands to work from other packages, so if you want to disable RVIZ, you can add `--no-rviz` to the end of the command. If you're working without the gripper, you can add `--no-gripper` to disable it <br/>

#### Realsense
The realsense ROS node is needed for the camera topics to be accessed.
In a new terminal tab, attach to the container (`./docker-attach.sh`) <br/>
Run:
```sh
realsense_driver
```
To test the camera display, open a new tab and open RVIZ. Then, add an Image display and set the topic to `/camera/color/raw`. You should see a camera output.

### Creating new Packages
To create a new package, attach to the docker, and go into `~/workspace/src` and run the package create command. Make sure to run your build commands in `~/workspace` and not `src`

### Shutting down
#### Stopping the container
`./docker-stop.sh` will shut down the container, but leave it for future use. It can be restarted with `./docker-start.sh`
#### Destroying the container
If you need to rebuild the container, you should run `./docker-delete.sh`, which will wipe the container and require a rebuild to use again. You shouldn't have to do this very often.


## Credits
*TODO Format nicely*
Sam/Jasper
Some ros wiki pages, will find and add when I get a chance.
