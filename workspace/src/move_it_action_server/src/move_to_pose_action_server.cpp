#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// #include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose.hpp"
#include "par_interfaces/action/set_pose.hpp"

 
class MoveToPoseActionServer : public rclcpp::Node
{
public:
    using SetPose = par_interfaces::action::SetPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SetPose>;

    explicit MoveToPoseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("move_to_pose_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server = rclcpp_action::create_server<SetPose>;
        (
            this, // node
            "SetPose", //action name
            std::bind(&MoveToPoseActionServer::handle_goal, this, _1, _2), //goal callback
            std::bind(&MoveToPoseActionServer::handle_cancel, this, _1),
            std::bind(&MoveToPoseActionServer::handle_accepted, this, _1)
        );
    }

private:
    rclcpp_action::Server<SetPose>::SharedPtr action_server;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid
        /*std::shared_ptr<const SetPose::Goal> goal*/)
    {
        // RCLCPP_INFO(this->get_logger(), "Recieved goal request %", goal->goal_pose);

        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&MoveToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal(); 
        // auto feedback = std::make_shared<SetPose::Feedback>();

        auto result = std::make_shared<SetPose::Result>();

        static const std::string PLANNING_GROUP = "ur5e_manipulator";

        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        geometry_msgs::msg::Pose target_pose = goal;

        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan_msg;

        bool plan_success = move_group.plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS;

        goal_handle->publish_feedback(plan_success);
        RCLCPP_INFO(this->get_logger(), "Plan success");


        if(plan_success)
        {
            if(move_group.move())
            {
                
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            } else
            {
                RCLCPP_ERROR(this->get_logger(), "Goal failed");

            }
            
        } else
        {
            RCLCPP_ERROR(this->get_logger(), "Goal failed");

        }

        result->final_pose;
        goal_handle->succeed(result);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(MoveToPoseActionServer)
