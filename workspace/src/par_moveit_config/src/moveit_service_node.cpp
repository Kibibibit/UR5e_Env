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

  this->move_group_interface = new MoveGroupInterface(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
  this->move_group_interface->startStateMonitor();

}

MoveitServiceNode::~MoveitServiceNode() {
  delete this->move_group_interface;
}

void MoveitServiceNode::get_current_pose(const std::shared_ptr<CurrentMoveitPose::Request> request, const std::shared_ptr<CurrentMoveitPose::Response> response) {
  (void)request;
  response->pose = this->move_group_interface->getCurrentPose().pose;
}



int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<MoveitServiceNode> node = std::make_shared<MoveitServiceNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });
  spinner.join();
  rclcpp::shutdown();
  return 0;
}