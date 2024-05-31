

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node, Publisher
from .onrobot.onrobot import RG
from pymodbus.exceptions import ConnectionException 
from sensor_msgs.msg import JointState
from . import helpers as h
from par_interfaces.msg import GripperInfo

### gripper_state_publisher_node.py ###
# Author: Daniel Mills (s3843035@student.rmit.edu.au)
# Created: 2024-05-30
# Updated: 2024-05-31



GRIPPER_BUSY_BIT = 0
"""In the gripper status array, this is the index for the busy state of the gripper"""



UPPER_FINGER_JOINT = 0.785398
LOWER_FINGER_JOINT = -0.558505

class GripperStatePublisherNode(Node):

    def __init__(self):
        super().__init__('gripper_state_publisher_node')

        # Declare out parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gripperType', "rg2"),
                ('gripperIp', "10.234.6.47"),
                ('gripperPort', 502),
                ("gripperInfoPublishRate", 5),
                ("gripperInfoTopic", "/par/gripper/info"),
                ('gripperJointPublishRate', 100),
            ]
        )

        # Look-up parameters values
        self._gripper_type = self.get_parameter('gripperType').value
        """This is the type of gripper, and should be rg2 or rg6"""
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

    
        self._qos_profile = QoSProfile(depth=10)
        self._joint_publisher: Publisher = self.create_publisher(JointState, 'joint_states', self._qos_profile)

        self._info_publisher: Publisher = self.create_publisher(
            GripperInfo,
            self._gripper_info_topic,
            10 # TODO: Replace this with a launch parameter maybe?
        )

        self._gripper: RG = RG(self._gripper_type, self._gripper_ip, self._gripper_port)
        """This is our actual gripper object, all commands are sent to this"""
        try: 
            # Weirdly, calling self._gripper.open_connection() doesn't trigger a connection error.
            # Asking for any other method does though
            self._gripper.get_status()
        except ConnectionException: 
            self.get_logger().error("\033[31mFailed to connect to the gripper!. Please check arguments and the device's network connection and try again.\033[0m") # Red error print 
            exit() 
        
        self._gripper_max_width:float = self._gripper.max_width
        self._gripper_fingertip_offset = self._gripper.get_fingertip_offset()
        self._gripper_max_force: float = self._gripper.max_force
        """This is the maximum force that the gripper can exert, in newtons. Trying to go higher than this will result in the value being clamped to this amount."""
        
        

        self._gripper_joint_publish_timer = self.create_timer(1.0/self._gripper_joint_publish_rate, self.gripper_joint_publish_callback)
        """This timer will publish the joint state to update rviz/moveit"""

        self._gripper_info_timer = self.create_timer(1.0/self._gripper_info_publish_rate, self.gripper_info_callback)
        """This timer will publish info about the gripper now and then for other nodes if needed"""

        
    

    def close_connection(self):
        self._gripper.close_connection()

    def gripper_joint_publish_callback(self):

        gripper_width = self._gripper.get_width_with_offset()
        now = self.get_clock().now()

        
        delta: float = 1.0-(float(gripper_width)/float(self._gripper_max_width-self._gripper.get_fingertip_offset()))

        
        #TODO: Check this relationship is linear
        rotation: float = h.lerp(LOWER_FINGER_JOINT, UPPER_FINGER_JOINT, delta)

        joint_state: JointState = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["finger_joint"]
        joint_state.position= [rotation]

        self._joint_publisher.publish(joint_state)

    def gripper_info_callback(self):
        msg = GripperInfo()
        msg.gripper_type = self._gripper_type
        msg.port = str(self._gripper_port)
        msg.ip = self._gripper_ip
        msg.max_force = self._gripper_max_force
        msg.max_width = self._gripper_max_width
        msg.fingertip_offset = self._gripper_fingertip_offset
        self._info_publisher.publish(msg)

        

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

        