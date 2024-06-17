#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_action_server_node.hpp"
#include <functional>
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "helpers.hpp"

MoveitActionServerNode::MoveitActionServerNode(const rclcpp::NodeOptions& options)
  : Node("moveit_action_server_node", options)
  , node_(std::make_shared<rclcpp::Node>("moveit_action_server_node"))
  , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  using namespace std::placeholders;
  this->action_server = rclcpp_action::create_server<WaypointMove>(
      this, "par_moveit/waypoint_move", std::bind(&MoveitActionServerNode::handle_goal, this, _1, _2),
      std::bind(&MoveitActionServerNode::handle_cancel, this, _1),
      std::bind(&MoveitActionServerNode::handle_accepted, this, _1));


  move_plane_height = 0.45;

  executing_move = false;
  this->move_group_interface =
      std::make_shared<MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(node_), "ur_manipulator_end_effector");
  this->move_group_interface->setPlanningTime(5.0);
  this->move_group_interface->setNumPlanningAttempts(10);
  this->move_group_interface->setMaxVelocityScalingFactor(0.1);
  this->move_group_interface->setMaxAccelerationScalingFactor(0.1);
  this->executor_->add_node(node_);

  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

rclcpp_action::GoalResponse MoveitActionServerNode::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                std::shared_ptr<const WaypointMove::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with target_pose (XYZ: %f %f %f, R: %f)",
    goal->target_pose.position.x,
    goal->target_pose.position.y,
    goal->target_pose.position.z,
    goal->target_pose.rotation
  );
  (void)uuid;
  if (goal->target_pose.rotation > M_PI_2 || goal->target_pose.rotation < -M_PI_2) {
    RCLCPP_ERROR(this->get_logger(), "Rejecting goal: Rotation %f is outside range [%f, %f]", 
    goal->target_pose.rotation,
    -M_PI_2,
    M_PI_2
    );
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitActionServerNode::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal!");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitActionServerNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&MoveitActionServerNode::execute, this, _1), goal_handle }.detach();
}

void MoveitActionServerNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing Goal");

  std::vector<Pose> waypoints;

  std::shared_ptr<const WaypointMove::Goal> goal = goal_handle->get_goal();

  WaypointPose target_waypoint_pose = goal->target_pose;
  Pose target_pose = pose_from_waypoint_pose(target_waypoint_pose);
  WaypointPose current_waypoint_pose = waypoint_pose_from_pose(this->move_group_interface->getCurrentPose().pose);

  if (will_translate(current_waypoint_pose, target_waypoint_pose, 0.0001))
  {
    // Compute the path
    // Step 1 -> go to plane height
    WaypointPose step_1_pose;
    step_1_pose.position = current_waypoint_pose.position;
    step_1_pose.position.z = this->move_plane_height;
    step_1_pose.rotation = current_waypoint_pose.rotation;

    waypoints.push_back(pose_from_waypoint_pose(step_1_pose));

    // Step 2 -> Rotate to match target rotation, and translate to match x/y
    WaypointPose step_2_pose;
    step_2_pose.position = target_waypoint_pose.position;
    step_2_pose.position.z = this->move_plane_height;
    step_2_pose.rotation = target_waypoint_pose.rotation;

    waypoints.push_back(pose_from_waypoint_pose(step_2_pose));
  }

  waypoints.push_back(target_pose);

  rclcpp::Rate loop_rate(1);

  std::shared_ptr<WaypointMove::Feedback> feedback = std::make_shared<WaypointMove::Feedback>();
  std::shared_ptr<WaypointMove::Result> result = std::make_shared<WaypointMove::Result>();

  feedback->current_pose = current_waypoint_pose;
  result->final_pose = current_waypoint_pose;

  this->move_group_interface->setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;
  bool success = static_cast<bool>(
      this->move_group_interface->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory));

  std::thread execution_thread;

  if (success && rclcpp::ok() && !this->executing_move)
  {
    using namespace std::placeholders;
    execution_thread = std::thread{ std::bind(&MoveitActionServerNode::execute_plan, this, _1), trajectory };
  }
  else
  {
    goal_handle->abort(result);
    if (this->executing_move)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal failed as a move is already being executed!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Goal Failed!");
    }

    return;
  }

  while (rclcpp::ok() && this->executing_move)
  {
    current_waypoint_pose = waypoint_pose_from_pose(this->move_group_interface->getCurrentPose().pose);
    feedback->current_pose = current_waypoint_pose;
    if (goal_handle->is_canceling())
    {
      result->final_pose = current_waypoint_pose;
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      goal_handle->canceled(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publish feedback for %s", this->move_group_interface->getEndEffector().c_str());

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  execution_thread.join();

  if (rclcpp::ok())
  {
    current_waypoint_pose = waypoint_pose_from_pose(this->move_group_interface->getCurrentPose().pose);
    result->final_pose = current_waypoint_pose;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void MoveitActionServerNode::execute_plan(moveit_msgs::msg::RobotTrajectory trajectory)
{
  this->executing_move = true;
  this->move_group_interface->execute(trajectory);
  this->executing_move = false;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = std::make_shared<MoveitActionServerNode>(node_options);
  rclcpp::spin(move_group_node);

  rclcpp::shutdown();
  return 0;
}