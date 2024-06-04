#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "moveit_service_node.hpp"
#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>

void get_current_pose(MoveGroupInterface* move_group_interface,
                      const std::shared_ptr<CurrentMoveitPose::Request> request,
                      const std::shared_ptr<CurrentMoveitPose::Response> response)
{
  (void)request;
  response->pose = move_group_interface->getCurrentPose().pose;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
      "moveit_service_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  MoveGroupInterface* move_group_interface;
  rclcpp::Service<CurrentMoveitPose>::SharedPtr service;

  move_group_interface = new MoveGroupInterface(std::shared_ptr<rclcpp::Node>(node), "ur_manipulator");
  move_group_interface->startStateMonitor();

  using namespace std::placeholders;
  service = node->create_service<CurrentMoveitPose>("par_get_current_moveit_pose",
                                                    std::bind(&get_current_pose, move_group_interface, _1, _2));

  spinner.join();
  rclcpp::shutdown();

  delete move_group_interface;

  return 0;
}