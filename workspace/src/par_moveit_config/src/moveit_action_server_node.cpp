#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_action_server_node.hpp"
#include <functional>
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>


MoveitActionServerNode::MoveitActionServerNode(const rclcpp::NodeOptions & options) : Node(
  "moveit_action_server_node",
  options
), 
node_(std::make_shared<rclcpp::Node>("moveit_action_server_node")), 
executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
  
  using namespace std::placeholders;
  this->pose_action_server = rclcpp_action::create_server<MoveitPose>(
    this,
    "par_moveit_pose",
    std::bind(&MoveitActionServerNode::handle_goal<MoveitPose::Goal>, this, _1, _2),
    std::bind(&MoveitActionServerNode::handle_cancel<MoveitPose>, this, _1),
    std::bind(&MoveitActionServerNode::pose_handle_accepted, this, _1)
  );
  this->point_action_server = rclcpp_action::create_server<MoveitPoint>(
    this,
    "par_moveit_point",
    std::bind(&MoveitActionServerNode::handle_goal<MoveitPoint::Goal>, this, _1, _2),
    std::bind(&MoveitActionServerNode::handle_cancel<MoveitPoint>, this, _1),
    std::bind(&MoveitActionServerNode::point_handle_accepted, this, _1)
  );

  executing_move = false;

  this->move_group_interface = std::make_shared<MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(node_), "ur_manipulator");
  this->move_group_interface->setPlanningTime(5.0);
  this->move_group_interface->setNumPlanningAttempts(10);
  this->move_group_interface->setMaxVelocityScalingFactor(0.1);
  this->move_group_interface->setMaxAccelerationScalingFactor(0.1);
  this->executor_->add_node(node_);
  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });

}


template <typename T>
rclcpp_action::GoalResponse MoveitActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const T> goal
) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with target_pose");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename T>
rclcpp_action::CancelResponse MoveitActionServerNode::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle
) {
  RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal!");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitActionServerNode::pose_handle_accepted(
  const std::shared_ptr<GoalHandleMoveitPose> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&MoveitActionServerNode::execute<MoveitPose, MoveitPose::Feedback, MoveitPose::Result>, this, _1, _2), goal_handle, this->get_pose_from_pose(goal_handle)}.detach();
}

void MoveitActionServerNode::point_handle_accepted(
  const std::shared_ptr<GoalHandleMoveitPoint> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&MoveitActionServerNode::execute<MoveitPoint, MoveitPoint::Feedback, MoveitPoint::Result>, this, _1, _2), goal_handle, this->get_pose_from_point(goal_handle)}.detach();
}

template <typename T, typename F, typename R>
void MoveitActionServerNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle, Pose target_pose)
{

  

  RCLCPP_INFO(this->get_logger(), "Executing Goal");


  rclcpp::Rate loop_rate(1);
  

  auto feedback = std::make_shared<F>();

  Pose current_pose = this->move_group_interface->getCurrentPose().pose;

  feedback->current_pose = current_pose;
  auto result = std::make_shared<R>();
  result->final_pose = current_pose;

  RCLCPP_INFO(this->get_logger(), "Currently at pose: xyz: %f %f %f xyzw: %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w,

  );

  RCLCPP_INFO(this->get_logger(), "Going to pose: xyz: %f %f %f xyzw: %f %f %f %f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w,

  );

  MoveGroupInterface::Plan plan;
  this->move_group_interface->setStartStateToCurrentState();
  this->move_group_interface->setPoseTarget(target_pose);
  bool success = static_cast<bool>(this->move_group_interface->plan(plan));


  std::thread execution_thread;

  if (success && rclcpp::ok() && !this->executing_move) {
    using namespace std::placeholders;
    execution_thread = std::thread{std::bind(&MoveitActionServerNode::execute_plan, this, _1), plan};
  } else {
    goal_handle->abort(result);
    if (this->executing_move){
      RCLCPP_ERROR(this->get_logger(), "Goal failed as a move is already being executed!");
    } else{
      RCLCPP_ERROR(this->get_logger(), "Goal Failed!");
    }
   
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

geometry_msgs::msg::Pose MoveitActionServerNode::get_pose_from_pose(std::shared_ptr<GoalHandleMoveitPose> goal_handle) {
  return goal_handle->get_goal()->target_pose;
}

geometry_msgs::msg::Pose MoveitActionServerNode::get_pose_from_point(std::shared_ptr<GoalHandleMoveitPoint> goal_handle) {
  Pose pose = this->move_group_interface->getCurrentPose().pose;
  pose.position = goal_handle->get_goal()->target_point;
  return pose;
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveitActionServerNode>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}