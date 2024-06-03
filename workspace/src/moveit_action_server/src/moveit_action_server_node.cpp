#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include "moveit_action_server_node.hpp"
#include <functional>
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>


MoveitActionServerNode::MoveitActionServerNode(const rclcpp::NodeOptions & options) : Node(
  "moveit_action_server_node",
  options
) {
  
  using namespace std::placeholders;
  this->action_server = rclcpp_action::create_server<MoveitPose>(
    this,
    "par_moveit_pose",
    std::bind(&MoveitActionServerNode::handle_goal, this, _1, _2),
    std::bind(&MoveitActionServerNode::handle_cancel, this, _1),
    std::bind(&MoveitActionServerNode::handle_accepted, this, _1)
  );

  this->move_group_interface = std::make_shared<MoveGroupInterface>(MoveGroupInterface(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator"));
  
}


rclcpp_action::GoalResponse MoveitActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveitPose::Goal> goal
) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with target_pose");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveitActionServerNode::handle_cancel(
  const std::shared_ptr<GoalHandleMoveitPose> goal_handle
) {
  RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal!");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitActionServerNode::handle_accepted(const std::shared_ptr<GoalHandleMoveitPose> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&MoveitActionServerNode::execute, this, _1), goal_handle}.detach();
}

void MoveitActionServerNode::execute(const std::shared_ptr<GoalHandleMoveitPose> goal_handle)
{

  RCLCPP_INFO(this->get_logger(), "Executing Goal");

  const auto goal = goal_handle->get_goal();

  rclcpp::Rate loop_rate(1);

  

  this->move_group_interface->setPoseTarget(goal->target_pose);

  auto feedback = std::make_shared<MoveitPose::Feedback>();
  feedback->current_pose = this->move_group_interface->getCurrentPose().pose;
  auto result = std::make_shared<MoveitPose::Result>();
  result->final_pose = this->move_group_interface->getCurrentPose().pose;


  MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(this->move_group_interface->plan(plan));

  std::thread execution_thread;

  if (success && rclcpp::ok() && !this->executing_move) {
    using namespace std::placeholders;
    execution_thread = std::thread{std::bind(&MoveitActionServerNode::execute_plan, this, _1), plan};
  } else {
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Goal Failed!");
    return;
  }

  while (rclcpp::ok() && this->executing_move) {
    feedback->current_pose = this->move_group_interface->getCurrentPose().pose;
    if (goal_handle->is_canceling()) {
      result->final_pose = feedback->current_pose;
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      goal_handle->canceled(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publish feedback for %s", this->move_group_interface->getEndEffector().c_str());

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  execution_thread.join();

  if (rclcpp::ok()) {
    result->final_pose = feedback->current_pose;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void MoveitActionServerNode::execute_plan(moveit::planning_interface::MoveGroupInterface::Plan plan) {
  this->executing_move = true;
  this->move_group_interface->execute(plan);
  this->executing_move = false;
}



int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitActionServerNode>());
  rclcpp::shutdown();
  return 0;
}