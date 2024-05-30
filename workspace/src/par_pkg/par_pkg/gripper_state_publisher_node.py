

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node, Publisher
from .onrobot.onrobot import RG
from pymodbus.exceptions import ConnectionException 

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

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

        self._qos_profile = QoSProfile(depth=10)
        self._joint_publisher: Publisher = self.create_publisher(JointState, 'joint_states', self._qos_profile)
        #self._joint_broadcaster: TransformBroadcaster = TransformBroadcaster(self, qos=self._qos_profile)

        self._gripper: RG = RG(self._gripper_type, self._gripper_ip, self._gripper_port)
        """This is our actual gripper object, all commands are sent to this"""
        try: 
            # Weirdly, calling self._gripper.open_connection() doesn't trigger a connection error.
            # Asking for any other method does though
            self._gripper.get_status()
        except ConnectionException: 
            self.get_logger().error("\033[31mFailed to connect to the gripper!. Please check arguments and the device's network connection and try again.\033[0m") # Red error print 
            exit() 
        
        self._gripper_max_width:float = (self._gripper.max_width-self._gripper.get_fingertip_offset())/10.0

        self._gripper_joint_publish_timer = self.create_timer(1.0/self._gripper_joint_publish_rate, self.gripper_joint_publish_callback)
        """This timer will publish the joint state to update rviz/moveit"""
    

    def close_connection(self):
        self._gripper.close_connection()

    def gripper_joint_publish_callback(self):

        gripper_width = self._gripper.get_width_with_offset()
        now = self.get_clock().now()

        
        delta: float = 1.0-(float(gripper_width)/float(self._gripper_max_width))

        

        rotation: float = (1.0 - delta)*LOWER_FINGER_JOINT + (delta*UPPER_FINGER_JOINT)

        self.get_logger().info(f"{gripper_width}-{delta}")

        joint_state: JointState = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["finger_joint"]
        joint_state.position= [rotation]

        self._joint_publisher.publish(joint_state)


        

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

        