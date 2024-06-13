from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
            executable='calibrate_camera_node',
            name='calibrate_camera_node',
            output='screen',
            parameters=[]
        )
        #Node(
         #   package='par_pkg',
          #  executable='move_to_pose_node',
           # name="move_to_pose_node",
            #output="screen",
            #parameters=[]
        #)
    ])