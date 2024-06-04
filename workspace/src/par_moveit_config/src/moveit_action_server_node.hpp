#ifndef PAR_MOVEIT_ACTION_SERVER_NODE_H
#define PAR_MOVEIT_ACTION_SERVER_NODE_H
#include <moveit/move_group_interface/move_group_interface.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "par_interfaces/action/moveit_pose.hpp"
#include "par_interfaces/action/moveit_point.hpp"
#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/point.h"
#include <functional>
#include <string>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class MoveitActionServerNode : public rclcpp::Node
{
    public:
        
        using Pose = geometry_msgs::msg::Pose;
        using Point = geometry_msgs::msg::Point;
        using MoveitPose = par_interfaces::action::MoveitPose;
        using MoveitPoint = par_interfaces::action::MoveitPoint;
        using GoalHandleMoveitPose = rclcpp_action::ServerGoalHandle<MoveitPose>;
        using GoalHandleMoveitPoint = rclcpp_action::ServerGoalHandle<MoveitPoint>;
        MoveitActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


    private:

        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        std::thread executor_thread_;
        std::string node_namespace_;

        moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
        
        std::atomic<bool> executing_move;

        rclcpp_action::Server<MoveitPose>::SharedPtr pose_action_server;
        rclcpp_action::Server<MoveitPoint>::SharedPtr point_action_server;

        template <typename T>
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const T> goal
        );

        void execute_plan(MoveGroupInterface::Plan plan);

        template <typename T>
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle);
        
        void pose_handle_accepted(const std::shared_ptr<GoalHandleMoveitPose> goal_handle);
        void point_handle_accepted(const std::shared_ptr<GoalHandleMoveitPoint> goal_handle);

        template <typename T, typename F, typename R>
        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle, Pose target_pose);

        Pose get_pose_from_pose(std::shared_ptr<GoalHandleMoveitPose> goal_handle);
        Pose get_pose_from_point(std::shared_ptr<GoalHandleMoveitPoint> goal_handle);

        


};

int main(int argc, char * argv[]);



#endif