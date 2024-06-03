#ifndef PAR_MOVEIT_SERVICE_NODE_H
#define PAR_MOVEIT_SERVICE_NODE_H
#include <moveit/move_group_interface/move_group_interface.h>
#include "rclcpp/rclcpp.hpp"
#include "par_interfaces/srv/current_moveit_pose.hpp"


class MoveitServiceNode : public rclcpp::Node
{
    public:
        using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
        using CurrentMoveitPose = par_interfaces::srv::CurrentMoveitPose;
        MoveitServiceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:

        std::shared_ptr<MoveGroupInterface> move_group_interface;
        rclcpp::Service<CurrentMoveitPose>::SharedPtr service;
        
        void get_current_pose(const std::shared_ptr<CurrentMoveitPose::Request> request, const std::shared_ptr<CurrentMoveitPose::Response> response);


};

int main(int argc, char * argv[]);



#endif