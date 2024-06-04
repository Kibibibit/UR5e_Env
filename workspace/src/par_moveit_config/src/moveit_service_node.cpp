#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_service_node.hpp"
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>

MoveitServiceNode::MoveitServiceNode(const rclcpp::NodeOptions & options) : Node(
  "moveit_service_node",
  options
), 
node_(std::make_shared<rclcpp::Node>("moveit_service_group_node")), 
executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {


  using namespace std::placeholders;
  this->service= this->create_service<CurrentMoveitPose>(
    "par_get_current_moveit_pose",
    std::bind(&MoveitServiceNode::get_current_pose, this, _1, _2)
  );

  auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
            "ur_manipulator",
            "/",
            "robot_description");

  this->move_group_interface = std::make_shared<MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(node_), mgi_options);
  move_group_interface->setPlanningTime(5.0);
  move_group_interface->setNumPlanningAttempts(10);
  move_group_interface->setMaxVelocityScalingFactor(0.1);
  move_group_interface->setMaxAccelerationScalingFactor(0.1);
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { this->executor_->spin(); });

}


void MoveitServiceNode::get_current_pose(const std::shared_ptr<CurrentMoveitPose::Request> request, const std::shared_ptr<CurrentMoveitPose::Response> response) {
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