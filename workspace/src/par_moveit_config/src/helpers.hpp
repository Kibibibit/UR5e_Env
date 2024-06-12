#ifndef PAR_MOVEIT_HELPERS_H
#define PAR_MOVEIT_HELPERS_H

#include "geometry_msgs/msg/vector3.hpp"
#include "par_interfaces/msg/waypoint_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw);
geometry_msgs::msg::Vector3 rpy_from_quaternion(geometry_msgs::msg::Quaternion q);
geometry_msgs::msg::Pose pose_from_waypoint_pose(par_interfaces::msg::WaypointPose waypoint_pose);
par_interfaces::msg::WaypointPose waypoint_pose_from_pose(geometry_msgs::msg::Pose pose);
bool equal_approx(double a, double b, double margin);
bool will_translate(par_interfaces::msg::WaypointPose a, par_interfaces::msg::WaypointPose b, double margin);
double wrap_value(double min, double max, double value);

#endif