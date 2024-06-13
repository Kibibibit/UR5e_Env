from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def make_parameters(params: list[LaunchConfiguration]):
    out = []
    for param in params:
        out.append({str(param.variable_name):param})
    return out

def declare_node(name, params):
    return Node(
        package='onrobot_rg2_driver',
        executable=name,
        name=name,
        output='screen',
        parameters=make_parameters(params)
    )


def launch_setup(context, *args, **kwargs):
    gripper_ip = LaunchConfiguration('gripper_ip')
    gripper_port = LaunchConfiguration('gripper_port')
    gripper_check_rate = LaunchConfiguration('gripper_check_rate')
    gripper_joint_publish_rate = LaunchConfiguration("gripper_joint_publish_rate")
    gripper_info_publish_rate = LaunchConfiguration("gripper_info_publish_rate")

    control_node = declare_node(
        'gripper_control_node',
        [gripper_ip,gripper_port, gripper_check_rate]
    
    )

    state_publisher_node = declare_node(
        'gripper_state_publisher_node',
        [gripper_ip, gripper_port, gripper_check_rate, gripper_joint_publish_rate, gripper_info_publish_rate]
    )

    return [control_node, state_publisher_node]

def generate_launch_description():


    

    declared_launch_arguments = [
        DeclareLaunchArgument('gripper_ip', default_value='10.234.6.47'),
        DeclareLaunchArgument('gripper_port', default_value="502"),

        DeclareLaunchArgument('gripper_check_rate', default_value='50'),
        DeclareLaunchArgument('gripper_joint_publish_rate', default_value="100"),
        DeclareLaunchArgument("gripper_info_publish_rate", default_value="5")
    ]

   

    return LaunchDescription(declared_launch_arguments + [OpaqueFunction(function=launch_setup)])