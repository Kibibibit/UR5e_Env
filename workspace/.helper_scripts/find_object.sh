#!/bin/bash


ros2 launch find_object_2d ros2/find_object_3d.launch.py \
    gui:=true \
    objects_path:="/home/rosuser/workspace/src/par_pkg/objects/" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/info