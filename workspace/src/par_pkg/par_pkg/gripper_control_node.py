import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node, Publisher
from .onrobot.onrobot import RG
from par_interfaces.action import GripperSetWidth
from par_interfaces.msg import GripperInfo
from rclpy.action.server import ServerGoalHandle
from . import helpers as h
from pymodbus.exceptions import ConnectionException 

### gripper_control_node.py ###
# Author: Daniel Mills (s3843035@student.rmit.edu.au)
# Created: 2024-05-23
# Updated: 2024-05-23



GRIPPER_BUSY_BIT = 0
"""In the gripper status array, this is the index for the busy state of the gripper"""


### This is the node that will handle opening and closing, and other tasks for it.
### Contains:
###  - An action server for opening and closing the gripper
###  - A publisher for publishing gripper meta info (type, max width, max force and so on)
###  - TODO: A publisher for the joint state
###  - TODO: Possibly a publisher for the current gripper width?
###  - TODO: Possibly a publisher for the gripper status (eg. busy)

# TODO: Do something to account for fingertip offsets !!!! VERY IMPORTANT !!!!

# TODO: Also look into finger offsets and see what they actually do



class GripperControlNode(Node):

    def __init__(self):
        super().__init__('gripper_control_node')
        
        
        # Declare out parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gripperType', "rg2"),
                ('gripperIp', "192.168.1.1"),
                ('gripperPort', :502:),
                ("gripperCheckRate", 10),
                ("gripperPrecisionEpsilon", 0.1),
                ("gripperInfoPublishRate", 5),
                ("gripperInfoTopic", "/par/gripper/info"),
                #('gripperJointPublishRate', 100),
            ]
        )
        
        # Look-up parameters values
        self._gripper_type = self.get_parameter('gripperType').value
        """This is the type of gripper, and should be rg2 or rg6"""
        self._gripper_ip = self.get_parameter('gripperIp').value
        """This is the ip address of the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_port = self.get_parameter('gripperPort').value
        """This is the port that the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_check_rate = self.get_parameter('gripperCheckRate').value
        """This is how often the node checks the current gripper state, in Hz"""
        self._gripper_precision_epsilon = self.get_parameter('gripperPrecisionEpsilon').value
        """This is how precisely the gripper will try to reach a target width. In mm"""
        self._gripper_info_publish_rate = self.get_parameter("gripperInfoPublishRate").value
        """This is how often the gripper will publish its info to [gripper_info_topic], in Hz"""
        self._gripper_info_topic = self.get_parameter("gripperInfoTopic").value
        """The topic that the gripper info will be published to."""
        #self._gripper_joint_publish_rate = self.get_parameter("gripperJointPublishRate").value
        #"""The frequency in Hz that this node will update the joint state for RViz/Moveit"""
        
        self._gripper: RG = RG(self._gripper_type, self._gripper_ip, self._gripper_port)
        """This is our actual gripper object, all commands are sent to this"""
        try: 
            # Weirdly, calling self._gripper.open_connection() doesn't trigger a connection error.
            # Asking for any other method does though
            self._gripper.get_status()
        except ConnectionException: 
            self.get_logger().error("\033[31mFailed to connect to the gripper!. Please check arguments and the device's network connection and try again.\033[0m") # Red error print 
            exit() 
        
        self._gripper_check_rate_object = self.create_rate(self._gripper_check_rate)
        
        self._current_gripper_width = self.get_gripper_width()
        """This is the current width of the gripper, updated every [gripper_check_rate] seconds. In metres"""
        self._is_gripper_busy = self.get_gripper_is_busy()
        """This is true if the gripper is currently preforming a move, and cannot take new instructions. Updated every [gripper_check_rate] seconds"""
        
        self._max_width = self.milimetre_tenths_to_metres(self._gripper.max_width)
        """This is the maximum width in metres the gripper can open. Trying to open wider than this will result in the gripper being clamped to this size"""
        self._max_force = self.centinewtons_to_newtons(self._gripper.max_force)
        """This is the maximum force that the gripper can exert, in newtons. Trying to go higher than this will result in the value being clamped to this amount."""
        
        
        self._state_update_timer = self.create_timer(1.0/self._gripper_check_rate, self.state_update_timer_callback)
        """This timer will update this node's values for the gripper's width and busy status"""
        
        self._gripper_info_timer = self.create_timer(1.0/self._gripper_info_publish_rate, self.gripper_info_callback)
        """This timer will publish info about the gripper now and then for other nodes if needed"""
        
        #self._gripper_joint_publish_timer = self.create_timer(1.0/self._gripper_joint_publish_rate, self.gripper_joint_publish_callback)
        #"""This timer will publish the joint state to update rviz/moveit"""
        
        
        self._info_publisher: Publisher = self.create_publisher(
            GripperInfo,
            self._gripper_info_topic,
            10 # TODO: Replace this with a launch parameter maybe?
        )
        
        self._action_server = ActionServer(
            self,
            GripperSetWidth,
            'set_gripper_width',
            self.execute_callback
        )
        
        self.get_logger().info(f"Gripper Action Node starting on host: {self._gripper_ip}:{self._gripper_port}. TYPE = {self._gripper_type}")
        

    def state_update_timer_callback(self):
        self._current_gripper_width = self.get_gripper_width()
        self._is_gripper_busy = self.get_gripper_is_busy()
    
    def gripper_info_callback(self):
        msg = GripperInfo()
        msg.gripper_type = self._gripper_type
        msg.port = str(self._gripper_port)
        msg.ip = self._gripper_ip
        msg.max_force = self._max_force
        msg.max_width = self._max_width
        self._info_publisher.publish(msg)
        

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing gripper action!')
        if (self._is_gripper_busy):
            goal_handle.abort()
            self.get_logger().error("Tried to move gripper while it was busy! Aborting action!")
            return
        
        target_width = goal_handle.request.target_width
        target_force = goal_handle.request.target_force
        if (target_width > self._max_width or target_width < 0):
            self.get_logger().warn(f"Tried to set gripper width to {target_width}m which is outside range [0,{self._max_width}]. Clamping!")
            target_width = h.clamp(0, self._max_width, target_width)
        if (target_force > self._max_force or target_force < 0):
            self.get_logger().warn(f"Tried to set gripper force to {target_force} which is outside range [0,{self._max_force}]. Clamping!")
            target_force = h.clamp(0, self._max_force, target_force)
        
        
        self._gripper.move_gripper(
            int(self.metres_to_milimetre_tenths(target_width)), 
            int(self.newtons_to_centinewtons(target_force))
        )
        
        while(not self.gripper_reached_target(self._current_gripper_width, target_width) or self._is_gripper_busy):
            feedback_msg = GripperSetWidth.Feedback()
            feedback_msg.current_width = self._current_gripper_width
            goal_handle.publish_feedback(feedback_msg)
            self._gripper_check_rate_object.sleep()
        
        self.get_logger().info("Gripper move suceeded!")
        
        goal_handle.succeed()
        result = GripperSetWidth.Result()
        result.final_width = self._current_gripper_width
        return result
    
    def gripper_reached_target(self, current: float, target:float) -> bool:
        return h.is_approx_equal(current, target, self._gripper_precision_epsilon)

    def get_gripper_width(self):
        return self.milimetre_tenths_to_metres(self._gripper.get_width())

    def get_gripper_status(self):
        return self._gripper.get_status()

    def get_gripper_is_busy(self) -> bool:
        """Returns true if the gripper is currently preforming an action"""
        return self.get_gripper_status()[GRIPPER_BUSY_BIT]

    def close_connection(self):
        self._gripper.close_connection()
        
    def centinewtons_to_newtons(self, v):
        return v/10.0
    def newtons_to_centinewtons(self, v):
        return v*10.0

    def metres_to_milimetre_tenths(self, v):
        return v*10000.0
    def milimetre_tenths_to_metres(self, v):
        return v/10000.0
    
    def gripper_joint_publish_callback(self):
        # TODO: Hook this up
        pass




def main(args=None):
    rclpy.init(args=args)

    # TODO: Make sure this is correct

    gripper_control_node = GripperControlNode()

    rclpy.spin(gripper_control_node)
    gripper_control_node.close_connection()


if __name__ == '__main__':
    main()