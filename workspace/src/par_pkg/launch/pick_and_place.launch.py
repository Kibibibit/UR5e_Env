from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='par_pkg',
            executable='move_to_pose_node',
            name="pick_and_place_node",
            output="screen",
            parameters=[]
        )
    ])