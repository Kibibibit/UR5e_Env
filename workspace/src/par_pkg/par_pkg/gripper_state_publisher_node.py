

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node, Publisher
from sensor_msgs.msg import JointState
from par_interfaces.msg import GripperInfo, GripperState
from .utils.rg2_client import RG2Client
import math

### gripper_state_publisher_node.py ###
# Author: Daniel Mills (s3843035@student.rmit.edu.au)
# Created: 2024-05-30
# Updated: 2024-06-09



### These come from blender, manually measuring things.
### Would have preferred to get real measurements from onrobot
### but could not find them and did not receive a reply asking on robot themselves
UPPER_FINGER_JOINT = 1.87361095202
LOWER_FINGER_JOINT = 3.18697121414

class GripperStatePublisherNode(Node):

    def __init__(self):
        super().__init__('gripper_state_publisher_node')

        # Declare out parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gripperIp', "10.234.6.47"),
                ('gripperPort', 502),
                ("gripperInfoPublishRate", 5),
                ("gripperInfoTopic", "/par/gripper/info"),
                ('gripperJointPublishRate', 100),
                ("gripperCheckRate", 50),
                ('gripperStateTopic',"/par/gripper")
            ]
        )

        # Look-up parameters values
        self._gripper_ip = self.get_parameter('gripperIp').value
        """This is the ip address of the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_port = self.get_parameter('gripperPort').value
        """This is the port that the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_joint_publish_rate = self.get_parameter("gripperJointPublishRate").value
        """The frequency in Hz that this node will update the joint state for RViz/Moveit"""
        self._gripper_info_publish_rate = self.get_parameter("gripperInfoPublishRate").value
        """This is how often the gripper will publish its info to [gripper_info_topic], in Hz"""
        self._gripper_info_topic = self.get_parameter("gripperInfoTopic").value
        """The topic that the gripper info will be published to."""
        self._gripper_check_rate = self.get_parameter('gripperCheckRate').value
        """This is how often the node checks the current gripper state, in Hz"""
        self._gripper_state_topic = self.get_parameter('gripperStateTopic').value
        """The topic that the current gripper state will be published to"""

    
        self._qos_profile = QoSProfile(depth=10)
        self._joint_publisher: Publisher = self.create_publisher(JointState, 'joint_states', self._qos_profile)
        self._state_publisher: Publisher = self.create_publisher(GripperState, self._gripper_state_topic, self._qos_profile)

        self._info_publisher: Publisher = self.create_publisher(
            GripperInfo,
            self._gripper_info_topic,
            self._qos_profile
        )
        self._gripper: RG2Client = RG2Client(self._gripper_ip, self._gripper_port)
        """This is our actual gripper object, all commands are sent to this"""
        if (not self._gripper.open_connection()):
            exit() 
        
        self._gripper_max_width:float = self._gripper.max_width
        self._gripper_fingertip_offset = self._gripper.get_fingertip_offset()
        self._gripper_max_force: float = self._gripper.max_force
        """This is the maximum force that the gripper can exert, in newtons. Trying to go higher than this will result in the value being clamped to this amount."""
        
        

        self._gripper_joint_publish_timer = self.create_timer(1.0/self._gripper_joint_publish_rate, self.gripper_joint_publish_callback)
        """This timer will publish the joint state to update rviz/moveit"""

        self._gripper_info_timer = self.create_timer(1.0/self._gripper_info_publish_rate, self.gripper_info_callback)
        """This timer will publish info about the gripper now and then for other nodes if needed"""

        self._gripper_state_timer = self.create_timer(1.0/self._gripper_check_rate, self.gripper_state_publish_callback)

        self.get_logger().info(f"Gripper State Publisher Node starting with gripper: {self._gripper_ip}:{self._gripper_port}.")
    

    def close_connection(self):
        self._gripper.close_connection()

    def gripper_joint_publish_callback(self):

        gripper_width = self._gripper.get_width()
        now = self.get_clock().now()

        if (gripper_width > 1):
            # This formula is based on a mathematical model that is very close to accurate but
            # not quite right. It should provide accurate enough information for RVIZ
            angle = math.pi - math.asin((gripper_width+0.7315)/114.3403)+0.038
        else:
            angle = LOWER_FINGER_JOINT


        joint_state: JointState = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["finger_joint"]
        joint_state.position= [angle]

        self._joint_publisher.publish(joint_state)

    def gripper_info_callback(self):
        msg = GripperInfo()
        msg.port = str(self._gripper_port)
        msg.ip = self._gripper_ip
        msg.max_force = self._gripper_max_force
        msg.max_width = self._gripper_max_width
        msg.fingertip_offset = self._gripper_fingertip_offset
        self._info_publisher.publish(msg)

    def gripper_state_publish_callback(self):
        msg = GripperState()
        msg.width = self._gripper.get_width_with_offset()
        msg.depth = self._gripper.get_actual_depth()
        msg.busy = self._gripper.get_busy()
        self._state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gripper_state_publisher_node = None
    try:
        gripper_state_publisher_node = GripperStatePublisherNode()

        rclpy.spin(gripper_state_publisher_node)
    except KeyboardInterrupt:
        pass
    gripper_state_publisher_node.close_connection()


if __name__ == '__main__':
    main()

        