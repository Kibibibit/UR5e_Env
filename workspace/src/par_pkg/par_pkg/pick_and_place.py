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

# enum determining the current action taking place
class ActionStates(Enum):
    OPEN_GRIPPER= 1
    MOVE_TO_ITEM = 2
    GRAB_ITEM = 3
    MOVE_ITEM_TO_NEW_POS = 4
    RELEASE_ITEM = 5
    
    WAIT = 0
    '''Waiting for goal request drom an action client'''

    DONE = 99
    '''Pick and place action is complete'''

FORCE:float = 10.0
''' default force to grab the object'''

FULL_OPEN_WIDTH:float = 70.0
'''Width the gripper should open to grab an item'''

RELEASE_WIDTH: float = 20.0
'''Width the gripper should open to release an item'''

UPDATE_RATE: float = 1.0
'''how often to check if an action is complete'''

class PickAndPlaceActionServer(Node):

    def __init__(self, action_callback_group, timer_callback_group):
        super().__init__('pick_and_place_node')

        # The action server to start the pick and place process
        self.pick_and_place_action_server = ActionServer(self, PickAndPlace, "/par/pick_and_place", self.pick_and_place_callback, callback_group=action_callback_group)

        # service client to get current pose
        self.current_pose_client = self.create_client(CurrentWaypointPose, "/par_moveit/get_current_waypoint_pose")

        # action client to open and close gripper at a certain width
        self.gripper_action_client = ActionClient(self, GripperSetWidth, "/rg2/set_width")

        # action client to move end effector to a new pose following cartesian path
        self.moveit_action_client = ActionClient(self, WaypointMove, "/par_moveit/waypoint_move")

        # the current state of the gripper
        self.gripper_state = GripperState()

        # subscriber to the gripper state node, which updates self.gripper_state
        self.create_subscription(GripperState, "/rg2/state", self.get_gripper_state,  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # feedback object of the pick and place node
        self.feedback = PickAndPlace.Feedback()

        # Sets the initial action to waiting for a goal request
        self.current_state = ActionStates.WAIT

        # This indicates if an action is currently in progress, if false, it will set the current action to the next in the sequence
        self.__action_in_progress:bool = True

        # timer callback that is called at a set rate that checks the current action and would switch positions
        self.create_timer(UPDATE_RATE, self.check_current_state, callback_group=timer_callback_group)
    
    def get_gripper_state(self, gp:GripperState ):
        # update gripper state whenever gripper publishes to topic /rg2/state
        self.gripper_state = gp

    # is called on request from an action client
    def pick_and_place_callback(self, goal_handle:ServerGoalHandle):
        # separating request parameters into separate variables 
        self.start_point = goal_handle.request.start_point 
        self.end_point = goal_handle.request.end_point
        self.gripper_width = goal_handle.request.gripper_width
        
        # This triggers the timer callback to switch to the first action (open gripper)
        self.__action_in_progress = False
        
        self.get_logger().info((f"Goal recieved with to move block from point {self.start_point.position.x}, {self.start_point.position.y}, {self.start_point.position.z}, {self.start_point.rotation}) to ({self.end_point.position.x}, {self.end_point.position.y}, {self.end_point.position.z}, {self.end_point.rotation}) {self.gripper_width=}"))

        # publish feedback until all actions are completed (current action in ActionStates is DONE)
        while (self.current_state != ActionStates.DONE):
            feedback = PickAndPlace.Feedback()
            feedback.current_pose = self.get_current_pose()
            feedback.gripper_current_width = self.gripper_state.width
            goal_handle.publish_feedback(feedback)
            time.sleep(0.25)
        
        # set the current action to wait for new request
        self.current_state = ActionStates.WAIT
        self.__action_in_progress:bool = True

        # send the request result to the client
        goal_handle.succeed()
        result = PickAndPlace.Result()
        result.final_arm_pose = self.get_current_pose()
        result.gripper_final_width = self.gripper_state.width
        return result


    def check_current_state(self):
        if(self.__action_in_progress):
            # uncomment logger if needed, was polluting the logs in full package
            # self.get_logger().info(f"completing action {self.current_state.name}...")
            pass
        else:
            # checkswhat action completed and sets the current action to the next action
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
        # set the current action and send request to gripper action server
        self.current_state = ActionStates.OPEN_GRIPPER
        self.move_gripper(FULL_OPEN_WIDTH, 40.0)

    def move_to_item(self):
        # set the current action and send request to moveit action server
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
        # set gripper width and force as a goal
        move_gripper_goal = GripperSetWidth.Goal()

        move_gripper_goal.target_force = force
        move_gripper_goal.target_width = width

        # wait until server is ready
        self.gripper_action_client.wait_for_server()

        # send request and once a response is recieved start the response callback 
        gripper_future = self.gripper_action_client.send_goal_async(move_gripper_goal, feedback_callback= self.gripper_feedback)
        gripper_future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future:Future):
        gripper_goal:ClientGoalHandle = future.result()

        # check if the result is appected or rejected
        if not gripper_goal.accepted:
            self.get_logger().info(f"Gripper Rejected Goal") 
            return
        
        self.get_logger().info(f"Gripper Accepted Goal")

        # get the result on response
        gripper_action_result_future:Future = gripper_goal.get_result_async()
        gripper_action_result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future:Future):
        # display result
        result:GripperSetWidth.Result = future.result().result
        self.get_logger().info(f"Gripper set width at: {result.final_width}")

        # state the action has completed
        self.__action_in_progress = False
    
    def gripper_feedback(self, gripper_feedback):
        # normally would just log feedback but to keep logs readable in the one console, this has been removed
        # add logs if debugging is needed 
        # self.get_logger().info(f"current gripper width: {gripper_feedback.feedback.current_width}") 
        pass
        
    def move_to_pose(self, pose:WaypointPose):
        # send pose goal to action moveit server
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
        # normally would just log feedback but to keep logs readable in the one console, this has been removed
        # add logs if debugging is needed 
        pass #self.get_logger().info(f"current pose: {moveit_feedback.feedback.current_pose}")
       
    def get_current_pose(self):
        # request the current pose from service
        current_pose_future = self.current_pose_client.call_async(CurrentWaypointPose.Request())

        # wait until service responds
        while not current_pose_future.done():
            pass

        # return current result
        current_pose:CurrentWaypointPose.Response = current_pose_future.result()
        return current_pose.pose
              
def main(args=None):
    rclpy.init(args=args)

    # added callback groups to allow simultaneous execution 
    # of timer callback and action server callback 
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

