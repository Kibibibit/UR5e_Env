// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_
#define PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "par_interfaces/msg/detail/gripper_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace par_interfaces
{

namespace msg
{

namespace builder
{

class Init_GripperInfo_max_force
{
public:
  explicit Init_GripperInfo_max_force(::par_interfaces::msg::GripperInfo & msg)
  : msg_(msg)
  {}
  ::par_interfaces::msg::GripperInfo max_force(::par_interfaces::msg::GripperInfo::_max_force_type arg)
  {
    msg_.max_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::msg::GripperInfo msg_;
};

class Init_GripperInfo_max_width
{
public:
  explicit Init_GripperInfo_max_width(::par_interfaces::msg::GripperInfo & msg)
  : msg_(msg)
  {}
  Init_GripperInfo_max_force max_width(::par_interfaces::msg::GripperInfo::_max_width_type arg)
  {
    msg_.max_width = std::move(arg);
    return Init_GripperInfo_max_force(msg_);
  }

private:
  ::par_interfaces::msg::GripperInfo msg_;
};

class Init_GripperInfo_port
{
public:
  explicit Init_GripperInfo_port(::par_interfaces::msg::GripperInfo & msg)
  : msg_(msg)
  {}
  Init_GripperInfo_max_width port(::par_interfaces::msg::GripperInfo::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_GripperInfo_max_width(msg_);
  }

private:
  ::par_interfaces::msg::GripperInfo msg_;
};

class Init_GripperInfo_ip
{
public:
  explicit Init_GripperInfo_ip(::par_interfaces::msg::GripperInfo & msg)
  : msg_(msg)
  {}
  Init_GripperInfo_port ip(::par_interfaces::msg::GripperInfo::_ip_type arg)
  {
    msg_.ip = std::move(arg);
    return Init_GripperInfo_port(msg_);
  }

private:
  ::par_interfaces::msg::GripperInfo msg_;
};

class Init_GripperInfo_gripper_type
{
public:
  Init_GripperInfo_gripper_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperInfo_ip gripper_type(::par_interfaces::msg::GripperInfo::_gripper_type_type arg)
  {
    msg_.gripper_type = std::move(arg);
    return Init_GripperInfo_ip(msg_);
  }

private:
  ::par_interfaces::msg::GripperInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::msg::GripperInfo>()
{
  return par_interfaces::msg::builder::Init_GripperInfo_gripper_type();
}

}  // namespace par_interfaces

#endif  // PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__BUILDER_HPP_
