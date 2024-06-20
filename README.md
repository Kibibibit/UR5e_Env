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



## Drivers and Aliases
The docker container contains several drivers and aliases for running different parts of the codebase.
- `arm_drivers` - This runs the arm, gripper and camera drivers.
- `moveit_config_driver` - This runs the moveit drivers, along with our custom moveit action server.
- `find_object_2d` - Boots findobject2d, using its findobject3d launch file.
- `realsense_driver` - This boots just the camera driver, if needed.

All of these aliases are set in `workspace/testing_scripts/helper-aliases.sh`, and can be modified or added to there.

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
#### UR Controller and Gripper Controller
First, on the UR5e teach pendant, select: `Open>>Installation>>RemoteRos`. If asked to update the program, select `Update Program`.<br/>
Then, go to the `Program` tab, and under `URCaps`, add `External Control`. Make sure the IP matches the PC IP, and that the port is 50002.<br/>
Turn ON the arm, ensuring that the E-Stop is released.<br/>
Attach to the container (`./docker-attach.sh`) and run:
```sh
arm_drivers
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

### Creating new Packages
To create a new package, attach to the docker, and go into `~/workspace/src` and run the package create command. Make sure to run your build commands in `~/workspace` and not `src`

### Shutting down
#### Stopping the container
`./docker-stop.sh` will shut down the container, but leave it for future use. It can be restarted with `./docker-start.sh`
#### Destroying the container
If you need to rebuild the container, you should run `./docker-delete.sh`, which will wipe the container and require a rebuild to use again. You shouldn't have to do this very often.

### Playing Connect4 Package for UR5e Cobot
#### Setup
1. The board is found in `/workspace/src/par_pkg/objects` under the filename `0.png`. Ensure that the board is printed on **A3 paper** with a **margin width of 0**.

2. Secure board on a table to ensure movement cannot occur during gameplay

3. Start docker environment and attach to it

4. Ensure the workspace is built using the command `build_workspace`

4. Start up UR5e and Moveit drivers

5. Move arm directly above board with camera facing down. The best position is with camera having a birds eye view of the top row with the **Human Drop Zone** and **Robot Piece**. This can be achieved directly using the teach pendant or using the shell script `waypoint_move.sh` under `~/workspace/testing_scripts` <br><br>**Note: waypoint_move.sh requires arguments of position x, y, z and roation of the end effector which are all floats.** This can be ran as `./waypoint_move.sh <X> <Y> <Z> <rotation>` an example of this would be `./waypoint_move.sh -0.5 0 0.2 0`
6. Launch Find Object 2D using the command `find_object_2d` and ensure that the detecting the board object (will state that object is detected in the console)

7. Launch the Connect4 program by running the command `ros2 launch par_pkg par_pkg.launch.py`
9. The node is up and ready when the console says it ahs found the grid and is waiting for the human piece

#### Playing
*This has been designed to play with game pieces made with 2x2 lego blocks that have been stacked to 2 blocks each game piece. 25 pieces per player is a suitable amount to play the game.*
1. To start playing place a block in one colour at the top of the column in the **Human Drop Zone** where you would like the piece to fall to
2. If the robot has detected the piece it will confirm in the console and display a three second countdown to start its process of moving the block
3. The robot will move the block to where it would fall to in a regular connect4 board
4. After this the console will say it is waiting for a robot piece. Place a game piece in the grid square labeled
**Robot Piece**
5. The arm upon recognition should grab the piece and move it to the AI's corresponding response. Execution is complete when the console is asking for the players turn.
6. Repeat steps 1-5 until someone wins in connect 4

#### Known Issues
- Robot is known to come across issues with precision to the the players piece and appears to stop on a randomly while moving the robots piece on a rare instance. The robot is able to mostly recover and still play the game.

- Issues also arise where it rarely has detected a piece in the wrong grid square when attempting, making the current state of the game incorrect. Game is able to continue, just with the player's piece in the wrong spot.

- Universal robots has a known bug where the connection to the PC will drop if no action is sent to the robot, which will stop the robots ability to move its arm. Restarting the connection on the pendant most of the time will fix this issue, however sometimes drivers have to be restarted.
 
- Moveit driver is also known to timeout and refuse any commands, in which case it may be needed to restart the stack

- Camera has a black spot on the depth sensor where it cannot see anything in that spot, possibly attributed to realsense driver reading the  stereo camera as mono. Move board to ensure that all grid squares are visible, viewing the image in rviz under the topic `/camera_image` can help achieve this. 


## Credits
- Sam Griffiths (RMIT)
- Jasper Avice Demay (RMIT)
- [FindObject2D](https://github.com/introlab/find-object)
- [Kiyokawa Takuya](https://github.com/takuya-ki/onrobot-rg)
