import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from par_interfaces.action import WaypointMove, PickAndPlace
from par_interfaces.action import GripperSetWidth, GripperFullOpen
from rclpy.action.server import ServerGoalHandle
from par_interfaces.msg import WaypointPose
from par_interfaces.srv import CurrentWaypointPose
from rclpy.task import Future
from functools import partial

class PickAndPlaceActionServer(Node):

    def __init__(self):
        super().__init__('pick_and_place_node')

        self.pick_and_place_action_server = ActionServer(self, PickAndPlace, "/par/pick_and_place", self.pick_and_place_callback)

        self.current_pose_client = self.create_client(CurrentWaypointPose, "par_moveit/get_current_waypoint_pose")

        self.gripper_action_client = ActionClient(self, GripperSetWidth, "gripper_set_width")
        self.full_open_gripper = ActionClient(self, GripperFullOpen, "gripper_full_open")
        self.moveit_action_client = ActionClient(self, WaypointMove, "/par_moveit/waypoint_move")
        
        self.feedback = PickAndPlace.Feedback()

        self.done=False

    def pick_and_place_callback(self, goal_handle:ServerGoalHandle):
        # Defining goals
        self.goal_server = goal_handle
        self.start_point = self.goal_server.request.start_point 
        self.end_point = self.goal_server.request.end_point
        self.gripper_width = self.goal_server.request.gripper_width
        
        self.get_logger().info((f"Goal recieved with to move block from point
                                ({self.start_point.position.x}, {self.start_point.position.y}, {self.start_point.position.z}, {self.start_point.rotation})
                                    to ({self.end_point.position.x}, {self.end_point.position.y}, {self.end_point.position.z}, {self.end_point.rotation}) {self.gripper_width=}"))
        
        
        # open gripper
        self.get_logger().info("Opening gripper...")
        self.open_gripper()
        
            # don't know if i need this
        # while (not self.done):
        #     self.feedback.current_pose = self.get_current_pose()
        #     self.goal_server.publish_feedback(self.feedback)

        #     time.sleep(1)
        
        self.goal_server.succeed()

        result = PickAndPlace.Result()
        result.final_arm_pose = self.final_pose
        # result.gripper_final_width = self.

        result_msg = f"({result.final_arm_pose.position.x}, {result.final_arm_pose.position.y}, {result.final_arm_pose.position.z}, {result.final_arm_pose.rotation}) {result.gripper_final_width=}"
        self.get_logger().info("final pos and gripper width: " + result_msg)

        return result
    
    def open_gripper(self):
        open_gripper_goal = GripperFullOpen.Goal()
        open_gripper_goal.target_force = float(20) #40?
        self.send_action(self.full_open_gripper, open_gripper_goal, self.move_to_item)
    
    def move_to_item(self, gripper_future_result:Future):
        result = gripper_future_result.result().result
        # self.feedback.gripper_current_width = result
        # self.feedback.current_pose = self.get_current_pose()
        # self.goal_server.publish_feedback(self.feedback)

        self.get_logger().info("Moving above the block...")
        start_goal_pos = WaypointMove.Goal()
        start_goal_pos.target_pose = self.start_point
        self.send_action(self.moveit_action_client, start_goal_pos, self.grab_item)

    def grab_item(self, arm_future_result:Future):
        # result = arm_future_result.result().result
        # self.feedback.gripper_current_width = same gripper width as before
        # self.feedback.current_pose = result
        # self.goal_server.publish_feedback(self.feedback)

        self.get_logger().info("Grabbing block...")
        goal_gripper_width = GripperSetWidth.Goal()
        goal_gripper_width.target_width = self.gripper_width
        goal_gripper_width.target_force = 20.0#?
        self.send_action(self.gripper_action_client, goal_gripper_width, self.move_item)


    def move_item(self, gripper_future_result:Future):
        # self.feedback.gripper_current_width = gripper_future_result.result().result.final_width
        # self.feedback.current_pose = self.get_current_pose()
        # self.goal_server.publish_feedback(self.feedback)

        self.get_logger().info("Moving block to new pos...")  
        end_goal_pos = WaypointMove.Goal()
        end_goal_pos.target_pose = self.end_point
        self.send_action(self.moveit_action_client, end_goal_pos, self.release_item)
    
    def release_item(self, arm_future_result:Future):
        result = arm_future_result.result().result
        # self.feedback.current_pose = arm_future_result.result().result.final_pose
        # self.goal_server.publish_feedback(self.feedback)

        self.final_pose = result

        self.get_logger().info("Releasing block...")
        open_gripper_goal = GripperFullOpen.Goal()
        open_gripper_goal.target_force = float(20)
        self.send_action(self.full_open_gripper, open_gripper_goal, self.finish)

    def finish(self, gripper_future_result:Future):
        result = gripper_future_result.result().result
        self.done = True

    def send_action(self, action_client:ActionClient, goal_msg, method):
        action_client.wait_for_server()
        self.send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback= self.feedback_callback)
        self.send_goal_future.add_done_callback(partial(self.goal_response_callback, method=method))

    def goal_response_callback(self, future:Future, method):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted")

        self.get_action_result_future = goal_handle.get_result_async()
        if(method is not None):
            self.get_action_result_future.add_done_callback(method)

    # not sure about feedback here
    def feedback_callback(self, feedback_msg):
        pass
        # feedback = feedback_msg.
        # self.get_logger().info(feedback)

    def get_current_pose(self) -> WaypointPose:
        current_pose_future = self.current_pose_client.call_async(CurrentWaypointPose.Request())

        if current_pose_future.done():
            try:
                current_pose = current_pose_future.result().result.final_pose
                return current_pose

            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))
               
def main(args=None):
   rclpy.init(args=args)

   node = PickAndPlaceActionServer()

   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass

   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()    