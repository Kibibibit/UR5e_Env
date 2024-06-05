#ifndef PAR_MOVEIT_ACTION_SERVER_NODE_H
#define PAR_MOVEIT_ACTION_SERVER_NODE_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "par_interfaces/action/waypoint_move.hpp"
#include "par_interfaces/msg/waypoint_pose.hpp"
#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/point.h"
#include <functional>
#include <string>

//TODO: Make these launch params
#define EEF_STEP 0.06
#define JUMP_THRESHOLD 6.0

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;

using WaypointPose = par_interfaces::msg::WaypointPose;

class MoveitActionServerNode : public rclcpp::Node
{
    public:
        

        using WaypointMove = par_interfaces::action::WaypointMove;
        using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointMove>;
        MoveitActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:

        double move_plane_height;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        std::string node_namespace_;

        moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
        
        std::atomic<bool> executing_move;

        rclcpp_action::Server<WaypointMove>::SharedPtr action_server;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const WaypointMove::Goal> goal
        );
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
        void execute_plan(moveit_msgs::msg::RobotTrajectory trajectory);
        void execute(const std::shared_ptr<GoalHandle> goal_handle);

};

int main(int argc, char * argv[]);



#endif