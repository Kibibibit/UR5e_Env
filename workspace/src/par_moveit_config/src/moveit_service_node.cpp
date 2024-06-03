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
    std::bind(&MoveitServiceNode::get_current_pose, this, _1, _2)
  );

  this->move_group_interface = std::make_shared<MoveGroupInterface>(MoveGroupInterface(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator"));
  
}

void MoveitServiceNode::get_current_pose(const std::shared_ptr<CurrentMoveitPose::Request> request, const std::shared_ptr<CurrentMoveitPose::Response> response) {
  (void)request;
  response->pose = this->move_group_interface->getCurrentPose().pose;
  RCLCPP_INFO(this->get_logger(), "Returing pose in frame %s", this->move_group_interface->getCurrentPose().header.frame_id);
}


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitServiceNode>());
  rclcpp::shutdown();
  return 0;
}