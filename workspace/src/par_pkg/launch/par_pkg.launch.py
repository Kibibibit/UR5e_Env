from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        ### GRIPPER CONTROL NODE ARGUMENTS
        
        #### GRIPPER JOINT PUBLISHING SETUP
        DeclareLaunchArgument('gripperJointPublishRate', default_value="100"),
        
        #### GRIPPER SETUP
        DeclareLaunchArgument('gripperType', default_value="rg2"),
        DeclareLaunchArgument('gripperIp', default_value='10.234.6.47'),
        DeclareLaunchArgument('gripperPort', default_value="502"),
        
        #### GRIPPER BEHAVIOUR CONFIG
        DeclareLaunchArgument('gripperCheckRate', default_value='50'),
        DeclareLaunchArgument('gripperInfoPublishRate', default_value="5"),
        
        #### GRIPPER TOPICS
        DeclareLaunchArgument("gripperInfoTopic", default_value="/par/gripper/info"),
        
        Node(
            package='par_pkg',
            executable='gripper_control_node',
            name='gripper_control_node',
            output='screen',
            parameters=[
                {'gripperType': LaunchConfiguration('gripperType')},
                {'gripperIp': LaunchConfiguration('gripperIp')},
                {'gripperPort': LaunchConfiguration('gripperPort')},
                {'gripperCheckRate': LaunchConfiguration('gripperCheckRate')},
            ]
        ),
        Node(
            package='par_pkg',
            executable='gripper_state_publisher_node',
            name='gripper_state_publisher_node',
            output='screen',
            parameters=[
                {'gripperJointPublishRate': LaunchConfiguration("gripperJointPublishRate")},
                {'gripperType': LaunchConfiguration('gripperType')},
                {'gripperIp': LaunchConfiguration('gripperIp')},
                {'gripperInfoTopic': LaunchConfiguration("gripperInfoTopic")},
                {'gripperInfoPublishRate': LaunchConfiguration("gripperInfoPublishRate")},
                {'gripperPort': LaunchConfiguration('gripperPort')},
            ]
        ),
        #Node(
         #   package='par_pkg',
          #  executable='move_to_pose_node',
           # name="move_to_pose_node",
            #output="screen",
            #parameters=[]
        #)
    ])