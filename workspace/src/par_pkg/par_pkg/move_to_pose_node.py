import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionClient
from par_interfaces.action import ParMoveItPose

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class MoveToPoseNode(Node): 

    def __init__(self):
        super().__init__('move_to_pose_node')

        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose', #look into actual topic name on robot, its similar 
            self.pose_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.moveit_action_client = ActionClient(self, ParMoveItPose, 'par_moveit_pose')

    def pose_callback(self, pose_stamped: PoseStamped):
        goal_pose = Pose()

        goal_pose = pose_stamped.pose

        self.send_pose(goal_pose)

    def send_pose(self, goal_pose:Pose):
        goal_msg = ParMoveItPose.Goal()
        goal_msg.target_pose = goal_pose

        self.moveit_action_client.wait_for_server()
        self.send_goal_future = self.moveit_action_client.send_goal_async(goal_msg, feedback_callback= self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger("Goal rejected")
            return
        
        self.get_logger.info("Goal accepted")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger.info(f"Move finished at pose: {result.final_pose}")


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger.info(f"Current Pose:{feedback.current_pose}")



def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()