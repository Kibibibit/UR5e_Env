// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__TRAITS_HPP_
#define PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "par_interfaces/msg/detail/gripper_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace par_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: gripper_type
  {
    out << "gripper_type: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_type, out);
    out << ", ";
  }

  // member: ip
  {
    out << "ip: ";
    rosidl_generator_traits::value_to_yaml(msg.ip, out);
    out << ", ";
  }

  // member: port
  {
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << ", ";
  }

  // member: max_width
  {
    out << "max_width: ";
    rosidl_generator_traits::value_to_yaml(msg.max_width, out);
    out << ", ";
  }

  // member: max_force
  {
    out << "max_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: gripper_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_type: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_type, out);
    out << "\n";
  }

  // member: ip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ip: ";
    rosidl_generator_traits::value_to_yaml(msg.ip, out);
    out << "\n";
  }

  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << "\n";
  }

  // member: max_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_width: ";
    rosidl_generator_traits::value_to_yaml(msg.max_width, out);
    out << "\n";
  }

  // member: max_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::msg::GripperInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::msg::GripperInfo & msg)
{
  return par_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::msg::GripperInfo>()
{
  return "par_interfaces::msg::GripperInfo";
}

template<>
inline const char * name<par_interfaces::msg::GripperInfo>()
{
  return "par_interfaces/msg/GripperInfo";
}

template<>
struct has_fixed_size<par_interfaces::msg::GripperInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<par_interfaces::msg::GripperInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<par_interfaces::msg::GripperInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__TRAITS_HPP_
