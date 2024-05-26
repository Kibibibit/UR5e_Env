# Dockerfile for UR5e ROS2 Moveit environment
# Author:           Daniel Mills
# Base Dockerfile:  Sam Griffiths, Jasper Avice Demay
# Last Updated:     2024-05-26

# Get the base image
FROM ros:humble

# Make sure the UID matches your local user. Running
# it this through docker-compose should handle that.
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Some generic locale envs
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_AU.UTF-8
ENV LANGUAGE=en_AU:en
ENV LC_ALL=en_AU.UTF-8


# Adding the user, installing sudo and allowing said user  to use sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod  --uid $USER_UID $USERNAME && \
    groupmod --gid $USER_GID $USERNAME

# Setup the environment to attach to
ENV SHELL /bin/bash
USER ${USERNAME}

# Make sure to update our package lists and other packages
RUN sudo apt-get update
RUN sudo apt-get upgrade -y

# Install locales and generate en_AU.UTF-8 locale
RUN sudo apt-get update
RUN sudo apt-get install -y locales
RUN sudo locale-gen en_AU.UTF-8
RUN sudo update-locale LC_ALL=en_AU.UTF-8 LANG=en_AU.UTF-8

# Install basic utilities
RUN sudo apt-get update && sudo apt-get install -y nano curl software-properties-common git python3-pip

# Install ROS 2 packages
RUN sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-urdfdom-py ros-$ROS_DISTRO-ros-testing ros-$ROS_DISTRO-moveit-msgs ros-$ROS_DISTRO-graph-msgs ros-$ROS_DISTRO-rviz-visual-tools ros-$ROS_DISTRO-object-recognition-msgs ros-$ROS_DISTRO-geometric-shapes ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-control-msgs ros-$ROS_DISTRO-ompl ros-$ROS_DISTRO-ament-clang-format ros-$ROS_DISTRO-warehouse-ros ros-$ROS_DISTRO-generate-parameter-library ros-$ROS_DISTRO-backward-ros ros-$ROS_DISTRO-controller-manager-msgs ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-gripper-controllers ros-$ROS_DISTRO-joint-state-broadcaster ros-$ROS_DISTRO-joint-trajectory-controller ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-position-controllers ros-$ROS_DISTRO-eigen-stl-containers ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-random-numbers ros-$ROS_DISTRO-ruckig ros-$ROS_DISTRO-joint-state-publisher-gui

# Install Python dependencies
RUN sudo apt-get update && sudo apt-get install -y python3-rosdep python3-colcon-common-extensions python3-colcon-mixin python3-vcstool

# Install additional dependencies
RUN sudo apt-get update && sudo apt-get install -y freeglut3-dev libomp-dev libfcl-dev
RUN sudo apt-get update && sudo apt-get install -y python3-colcon-mixin

# Install move it and UR
RUN sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-ur

# Install RVIZ
RUN sudo apt-get update && sudo apt-get install -y ros-$ROS_DISTRO-rviz2

# Install python requirements
RUN pip install pymodbus==2.5.3


RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc



CMD ["/bin/bash"]