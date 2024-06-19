import time
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from par_interfaces.action import WaypointMove, PickAndPlace
from onrobot_rg2_msgs.action import GripperSetWidth
from onrobot_rg2_msgs.msg import GripperState
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from par_interfaces.srv import CurrentWaypointPose
from rclpy.task import Future
from par_interfaces.msg import WaypointPose
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

from enum import Enum

class ActionStates(Enum):
    OPEN_GRIPPER= 1
    MOVE_TO_ITEM = 2
    GRAB_ITEM = 3
    MOVE_ITEM_TO_NEW_POS = 4
    RELEASE_ITEM = 5
    WAIT = 0
    DONE = 99

FORCE:float = 10.0
FULL_OPEN_WIDTH:float = 40.0
RELEASE_WIDTH: float = 20.0
UPDATE_RATE: float = 1.0

class PickAndPlaceActionServer(Node):

    def __init__(self, action_callback_group, timer_callback_group):
        super().__init__('pick_and_place_node')

        self.pick_and_place_action_server = ActionServer(self, PickAndPlace, "/par/pick_and_place", self.pick_and_place_callback, callback_group=action_callback_group)

        self.current_pose_client = self.create_client(CurrentWaypointPose, "/par_moveit/get_current_waypoint_pose")

        self.gripper_action_client = ActionClient(self, GripperSetWidth, "/rg2/set_width")
        self.moveit_action_client = ActionClient(self, WaypointMove, "/par_moveit/waypoint_move")

        self.gripper_state = GripperState()

        self.create_subscription(GripperState, "/rg2/state", self.get_gripper_state,  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.feedback = PickAndPlace.Feedback()

        self.current_state = ActionStates.WAIT
        self.__action_in_progress:bool = True

        self.create_timer(UPDATE_RATE, self.check_current_state, callback_group=timer_callback_group)
    
    def get_gripper_state(self, gp:GripperState ):
        self.gripper_state = gp

    def pick_and_place_callback(self, goal_handle:ServerGoalHandle):
        # Defining goals
        self.start_point = goal_handle.request.start_point 
        self.end_point = goal_handle.request.end_point
        self.gripper_width = goal_handle.request.gripper_width

        self.__action_in_progress = False
        
        self.get_logger().info((f"Goal recieved with to move block from point {self.start_point.position.x}, {self.start_point.position.y}, {self.start_point.position.z}, {self.start_point.rotation}) to ({self.end_point.position.x}, {self.end_point.position.y}, {self.end_point.position.z}, {self.end_point.rotation}) {self.gripper_width=}"))

        while (self.current_state != ActionStates.DONE):
            feedback = PickAndPlace.Feedback()
            feedback.current_pose = self.get_current_pose()
            feedback.gripper_current_width = self.gripper_state.width
            goal_handle.publish_feedback(feedback)
            time.sleep(0.25)
        
        self.current_state = ActionStates.WAIT
        self.__action_in_progress:bool = True

        goal_handle.succeed()
        result = PickAndPlace.Result()
        result.final_arm_pose = self.get_current_pose()
        result.gripper_final_width = self.gripper_state.width
        return result


    def check_current_state(self):
        if(self.__action_in_progress):
            # self.get_logger().info(f"completing action {self.current_state.name}...")
            pass
        else:
            self.__action_in_progress = True
            self.get_logger().info(f"Changing state {self.current_state.name}...")

            if (self.current_state == ActionStates.WAIT):
                self.open_gripper()
            elif (self.current_state == ActionStates.OPEN_GRIPPER):
                self.move_to_item()
            elif (self.current_state == ActionStates.MOVE_TO_ITEM):
                self.close_gripper()
            elif (self.current_state == ActionStates.GRAB_ITEM):
                self.move_to_new_pose()
            elif (self.current_state == ActionStates.MOVE_ITEM_TO_NEW_POS):
                self.release_item()
            elif (self.current_state == ActionStates.RELEASE_ITEM):
                self.current_state = ActionStates.DONE
                

    def open_gripper(self):
        self.current_state = ActionStates.OPEN_GRIPPER
        self.move_gripper(FULL_OPEN_WIDTH, 40.0)

    def move_to_item(self):
        self.current_state = ActionStates.MOVE_TO_ITEM
        self.move_to_pose(self.start_point)

    def close_gripper(self):
        self.current_state = ActionStates.GRAB_ITEM
        self.move_gripper(self.gripper_width)

    def move_to_new_pose(self):
        self.current_state = ActionStates.MOVE_ITEM_TO_NEW_POS

        self.move_to_pose(self.end_point)

    def release_item(self):
        self.current_state = ActionStates.RELEASE_ITEM
        self.move_gripper(RELEASE_WIDTH)

    def move_gripper(self, width, force = FORCE):
        move_gripper_goal = GripperSetWidth.Goal()

        move_gripper_goal.target_force = force
        move_gripper_goal.target_width = width

        self.gripper_action_client.wait_for_server()
        gripper_future = self.gripper_action_client.send_goal_async(move_gripper_goal, feedback_callback= self.gripper_feedback)
        gripper_future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future:Future):
        gripper_goal:ClientGoalHandle = future.result()

        if not gripper_goal.accepted:
            self.get_logger().info(f"Gripper Rejected Goal") 
            return
        
        self.get_logger().info(f"Gripper Accepted Goal")

        gripper_action_result_future:Future = gripper_goal.get_result_async()
        gripper_action_result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future:Future):
        result:GripperSetWidth.Result = future.result().result

        self.__action_in_progress = False
    
    def gripper_feedback(self, gripper_feedback):
        pass
        
    def move_to_pose(self, pose:WaypointPose):
        goal_pose = WaypointMove.Goal()

        goal_pose.target_pose.position = pose.position
        goal_pose.target_pose.rotation = pose.rotation

        self.moveit_action_client.wait_for_server()
        send_goal_future = self.moveit_action_client.send_goal_async(goal_pose, feedback_callback= self.moveit_feedback)
        send_goal_future.add_done_callback(self.move_response_callback)

    def move_response_callback(self, future:Future):
        pose_goal:ClientGoalHandle = future.result()

        if not pose_goal.accepted:
            self.get_logger().info("Moveit rejected")
            return
        
        self.get_logger().info("Moveit accepted")

        pose_action_result_future:Future = pose_goal.get_result_async()
        pose_action_result_future.add_done_callback(self.move_result_callback)

    def move_result_callback(self, future:Future):
        result:WaypointMove.Result = future.result().result
        self.get_logger().info(f"Finished move point at: {result.final_pose}")

        self.__action_in_progress = False

    def moveit_feedback(self, moveit_feedback):
        pass #self.get_logger().info(f"current pose: {moveit_feedback.feedback.current_pose}")
       
    def get_current_pose(self):
        current_pose_future = self.current_pose_client.call_async(CurrentWaypointPose.Request())

        while not current_pose_future.done():
            pass

        current_pose:CurrentWaypointPose.Response = current_pose_future.result()
        return current_pose.pose
              
def main(args=None):
    rclpy.init(args=args)


    action_callback_group = MutuallyExclusiveCallbackGroup()

    node = PickAndPlaceActionServer(
        action_callback_group=action_callback_group,
        timer_callback_group=None
    )

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()    

