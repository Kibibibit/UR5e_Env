import rclpy
from rclpy.node import Node

from rclpy.qos import ReliabilityPolicy, QoSProfile

import time

from rclpy.logging import get_logger

#will need proper import path
from moveit.planning import MoveItPy
# from moveit2.moveit_py.moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


class MoveToPoseNode(Node): 

    def __init__(self):
        super().__init__('move_to_pose_node')

        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/topic', 
            self.pose_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )


    def plan_and_execute(
        self,
        robot,
        planning_component,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.get_logger().info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.get_logger().info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed")

        time.sleep(sleep_time)

    def pose_callback(self, pose: PoseStamped):
        # unsure where to put this
        # need to fill in parameters properly
        cobot = MoveItPy()
        cobot_arm = cobot.get_planning_component("")
        cobot_arm.set_goal_state(pose_stamped_msg=pose, pose_link="")

        # plan to goal
        self.plan_and_execute(cobot, cobot_arm, sleep_time=3.0)

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