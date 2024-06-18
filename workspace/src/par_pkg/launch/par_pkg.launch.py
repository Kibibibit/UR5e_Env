from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='par_pkg',
            executable='table_depth_image_node',
            name='table_depth_image_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='par_pkg',
            executable='main_controller_node',
            name='main_controller_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package="par_pkg",
            executable="board_transformer_node",
            name="board_transformer_node",
            output="screen",
            parameters=[]
        ),
        Node(
           package='par_pkg',
           executable='pick_and_place_node',
           name="pick_and_place_node",
            output="screen",
            parameters=[]
        ),
        Node(
            package='par_pkg',
            executable='cube_detection_node',
            name='cube_detection_node',
            output='screen',
            parameters=[]
        ),
        
    ])