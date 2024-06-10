import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionClient
from par_interfaces.action import WaypointMove
from par_interfaces.msg import WaypointPose
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

## base example of actioon client to work with
class MoveToPoseNode(Node): 

    def __init__(self):
        super().__init__('move_to_pose_node')

        self.moveit_action_client = ActionClient(self, WaypointMove, "/par_moveit/waypoint_move")

    def pose_callback(self, goal_point: WaypointPose):
        # goal_point = Point()

        # goal_point = point

        self.send_pose(goal_point)

    def send_pose(self, goal_point:WaypointPose):
        goal_msg = WaypointMove.Goal()

        goal_msg.target_pose.position = goal_point.position
        goal_msg.target_pose.rotation = goal_point.rotation

        # goal_msg = WaypointPose
        # goal_msg.target_pose = target

        self.moveit_action_client.wait_for_server()
        self.send_goal_future = self.moveit_action_client.send_goal_async(goal_msg, feedback_callback= self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Move finished at pose: {result.final_pose}")


    def feedback_callback(self, feedback_msg : WaypointMove):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current Pose:{feedback.current_pose}")



def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()