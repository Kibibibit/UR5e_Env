#include "helpers.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "par_interfaces/msg/waypoint_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"


geometry_msgs::msg::Pose pose_from_waypoint_pose(par_interfaces::msg::WaypointPose waypoint_pose)
{
  geometry_msgs::msg::Pose pose;

  pose.position = waypoint_pose.position;
  pose.orientation =  quaternion_from_rpy(M_PI, 0.0, waypoint_pose.rotation);
  return pose;
}

par_interfaces::msg::WaypointPose waypoint_pose_from_pose(geometry_msgs::msg::Pose pose)
{
  
  par_interfaces::msg::WaypointPose waypoint_pose;

  waypoint_pose.position = pose.position;
  waypoint_pose.rotation = rpy_from_quaternion(pose.orientation).z;
  return waypoint_pose;
}


geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{

  // Yes this is the code from the wikipedia example
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}
geometry_msgs::msg::Vector3 rpy_from_quaternion(geometry_msgs::msg::Quaternion q) {
  geometry_msgs::msg::Vector3 angles;

  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  //Roll
  angles.x = std::atan2(sinr_cosp, cosr_cosp);

//Pitch
  double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  angles.y = 2 * std::atan2(sinp, cosp) - M_PI / 2;
//Yaw
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.z = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

bool equal_approx(double a, double b, double margin) {
  return abs(b-a) < margin;
}

bool will_translate(par_interfaces::msg::WaypointPose a, par_interfaces::msg::WaypointPose b, double margin) {
  bool x_move = !equal_approx(a.position.x, b.position.x, margin);
  bool y_move = !equal_approx(a.position.y, b.position.y, margin);
  bool rotation = !equal_approx(a.rotation, b.rotation, margin);
  return x_move || y_move || rotation;
}