#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_service_node.hpp"
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include "helpers.hpp"

MoveitServiceNode::MoveitServiceNode(const rclcpp::NodeOptions & options) : Node(
  "moveit_service_node",
  options
), 
node_(std::make_shared<rclcpp::Node>("moveit_service_node")), 
executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {


  using namespace std::placeholders;
  this->waypoint_service= this->create_service<CurrentWaypointPose>(
    "par_moveit/get_current_waypoint_pose",
    std::bind(&MoveitServiceNode::get_current_waypoint_pose, this, _1, _2)
  );
  this->pose_service= this->create_service<CurrentPose>(
    "par_moveit/get_current_pose",
    std::bind(&MoveitServiceNode::get_current_pose, this, _1, _2)
  );

  this->move_group_interface = std::make_shared<MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(node_), "ur_manipulator_end_effector");
  this->move_group_interface->setPlanningTime(5.0);
  this->move_group_interface->setNumPlanningAttempts(10);
  this->move_group_interface->setMaxVelocityScalingFactor(0.1);
  this->move_group_interface->setMaxAccelerationScalingFactor(0.1);
  this->executor_->add_node(node_);
  this->executor_thread_ = std::thread([this]() { this->executor_->spin(); });

}


void MoveitServiceNode::get_current_waypoint_pose(const std::shared_ptr<CurrentWaypointPose::Request> request, const std::shared_ptr<CurrentWaypointPose::Response> response) {
  (void)request;
  response->pose = waypoint_pose_from_pose(this->move_group_interface->getCurrentPose().pose);
}

void MoveitServiceNode::get_current_pose(const std::shared_ptr<CurrentPose::Request> request, const std::shared_ptr<CurrentPose::Response> response) {
  (void)request;
  response->pose = this->move_group_interface->getCurrentPose().pose;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveitServiceNode>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}