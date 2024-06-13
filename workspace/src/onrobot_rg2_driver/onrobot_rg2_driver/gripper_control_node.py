import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from onrobot_rg2_msgs.action import GripperSetWidth, GripperFullClose, GripperFullOpen
from rclpy.action.server import ServerGoalHandle
from .utils import helpers as h
import time
from .utils.rg2_client import RG2Client

### gripper_control_node.py ###
# Author: Daniel Mills (s3843035@student.rmit.edu.au)
# Created: 2024-05-23
# Updated: 2024-06-07


### This is the node that will handle opening and closing, and other tasks for it.
### Contains:
###  - An action server for opening and closing the gripper to a set width
###  - Action servers for open and closing the gripper to its max and min width

class GripperControlNode(Node):

    def __init__(self):
        super().__init__('gripper_control_node')
        
        
        # Declare out parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gripper_ip', "10.234.6.47"),
                ('gripper_port', 502),
                ("gripper_check_rate", 50),
            ]
        )
        
        # Look-up parameters values
        self._gripper_ip = self.get_parameter('gripper_ip').value
        """This is the ip address of the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_port = self.get_parameter('gripper_port').value
        """This is the port that the gripper, that modbus will use to communicate with the gripper"""
        self._gripper_check_rate = self.get_parameter('gripper_check_rate').value
        """This is how often the node checks the current gripper state, in Hz"""

        self._gripper: RG2Client = RG2Client(self._gripper_ip, self._gripper_port)
        """This is our actual gripper object, all commands are sent to this"""

        if (not self._gripper.open_connection()):
            exit()
        
        
        self._current_gripper_width = self.get_gripper_width()
        """This is the current width of the gripper, updated every [gripper_check_rate] seconds. In milimetres"""
        self._is_gripper_busy = self.get_gripper_is_busy()
        """This is true if the gripper is currently preforming a move, and cannot take new instructions. Updated every [gripper_check_rate] seconds"""
        self._gripper_target_width = self._current_gripper_width
        """This is the target width the gripper is trying to acheive, in mm"""

        self._max_width: float = self._gripper.max_width
        """This is the maximum width in milimetres the gripper can open. Trying to open wider than this will result in the gripper being clamped to this size"""
        self._max_force: float = self._gripper.max_force
        """This is the maximum force that the gripper can exert, in newtons. Trying to go higher than this will result in the value being clamped to this amount."""
        
        
        self._state_update_timer = self.create_timer(1.0/self._gripper_check_rate, self.state_update_timer_callback)
        """This timer will update this node's values for the gripper's width and busy status"""
        
        
        self._set_width_action_server = ActionServer(
            self,
            GripperSetWidth,
            'rg2/set_width',
            self.execute_set_width_callback
        )

        self._close_action_server = ActionServer(
            self,
            GripperFullClose,
            'rg2/full_close',
            self.execute_close_callback
        )

        self._open_action_server = ActionServer(
            self,
            GripperFullOpen,
            'rg2/full_open',
            self.execute_open_callback
        )
        
        self.get_logger().info(f"Gripper Action Node starting with gripper: {self._gripper_ip}:{self._gripper_port}.")
        

    def state_update_timer_callback(self):
        self._current_gripper_width = self.get_gripper_width()
        self._is_gripper_busy = self.get_gripper_is_busy()

        

    def execute_set_width_callback(self, goal_handle: ServerGoalHandle):
        target_width = goal_handle.request.target_width
        return self.execute_gripper_move(target_width, goal_handle, GripperSetWidth)
    
    def execute_open_callback(self, goal_handle: ServerGoalHandle):
        return self.execute_gripper_move(self._max_width, goal_handle, GripperFullOpen)
    
    def execute_close_callback(self, goal_handle: ServerGoalHandle):
        return self.execute_gripper_move(0.0, goal_handle, GripperFullClose)
    
    def execute_gripper_move(
            self, 
            target_width, 
            goal_handle: ServerGoalHandle,
            action_type
        ):
        target_force = goal_handle.request.target_force
        self.get_logger().info(f'Executing gripper action! width={target_width}, force={target_force}')
        if (self._is_gripper_busy):
            goal_handle.abort()
            self.get_logger().error("Tried to move gripper while it was busy! Aborting action!")
            return
        
        if (target_width > self._max_width or target_width < 0):
            self.get_logger().warn(f"Tried to set gripper width to {target_width}m which is outside range [0,{self._max_width}]. Clamping!")
            target_width:float = h.clamp(0, self._max_width, target_width)
        if (target_force > self._max_force or target_force < 0):
            self.get_logger().warn(f"Tried to set gripper force to {target_force} which is outside range [0,{self._max_force}]. Clamping!")
            target_force:float = h.clamp(0, self._max_force, target_force)
        
        
        self._gripper.move_gripper_with_offset(target_width, target_force)
        self.state_update_timer_callback()
        while(self._is_gripper_busy):
            self.state_update_timer_callback()
            feedback_msg = action_type.Feedback()
            feedback_msg.current_width = self._current_gripper_width
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0/self._gripper_check_rate)
        
        self.get_logger().info("Gripper move suceeded!")
        
        self.state_update_timer_callback()
        result = action_type.Result()
        result.final_width = self._current_gripper_width
        goal_handle.succeed()
        return result
    


    def get_gripper_width(self)->int:
        return self._gripper.get_width_with_offset()

    def get_gripper_status(self):
        return self._gripper.get_status()

    def get_gripper_is_busy(self) -> bool:
        """Returns true if the gripper is currently preforming an action"""
        return self._gripper.get_busy()

    def close_connection(self):
        self._gripper.close_connection()




def main(args=None):
    rclpy.init(args=args)

    gripper_control_node = None
    try:
        gripper_control_node = GripperControlNode()

        rclpy.spin(gripper_control_node)
    except KeyboardInterrupt:
        pass
    gripper_control_node.close_connection()


if __name__ == '__main__':
    main()