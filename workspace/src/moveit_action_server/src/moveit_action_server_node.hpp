#ifndef PAR_MOVEIT_ACTION_SERVER_NODE_H
#define PAR_MOVEIT_ACTION_SERVER_NODE_H
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "par_interfaces/action/par_moveit_pose.hpp"


class MoveitActionServerNode : public rclcpp::Node
{
    public:
        using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
        using ParMoveitPose = par_interfaces::action::ParMoveitPose;
        using GoalHandleParMoveitPose = rclcpp_action::ServerGoalHandle<ParMoveitPose>;
        MoveitActionServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:

        std::shared_ptr<MoveGroupInterface> move_group_interface;
            

        rclcpp_action::Server<ParMoveitPose>::SharedPtr action_server;
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const ParMoveitPose::Goal> goal
        );

        void execute_plan(MoveGroupInterface::Plan plan);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleParMoveitPose> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleParMoveitPose> goal_handle);

        void execute(const std::shared_ptr<GoalHandleParMoveitPose> goal_handle);

        bool executing_move = false;


};

int main(int argc, char * argv[]);



#endif