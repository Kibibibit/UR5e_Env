#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_service_node.hpp"
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>


MoveitServiceNode::MoveitServiceNode(const rclcpp::NodeOptions & options) : Node(
  "moveit_service_node",
  options
) {
  
  using namespace std::placeholders;
  this->service= this->create_service<CurrentMoveitPose>(
    "par_get_current_moveit_pose",
    &MoveitServiceNode::get_current_pose
  );

  this->move_group_interface = std::make_shared<MoveGroupInterface>(MoveGroupInterface(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator"));
  
}

void MoveitServiceNode::get_current_pose(const std::shared_ptr<CurrentMoveitPose::Request> request, const std::shared_ptr<CurrentMoveitPose::Response> response) {
    response->pose = this->move_group_interface->getCurrentPose().pose;
}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitServiceNode>());
  rclcpp::shutdown();
  return 0;
}