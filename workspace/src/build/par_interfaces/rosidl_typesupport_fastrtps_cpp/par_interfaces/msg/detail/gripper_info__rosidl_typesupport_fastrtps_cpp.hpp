// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "par_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "par_interfaces/msg/detail/gripper_info__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace par_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_par_interfaces
cdr_serialize(
  const par_interfaces::msg::GripperInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_par_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  par_interfaces::msg::GripperInfo & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_par_interfaces
get_serialized_size(
  const par_interfaces::msg::GripperInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_par_interfaces
max_serialized_size_GripperInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace par_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_par_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, par_interfaces, msg, GripperInfo)();

#ifdef __cplusplus
}
#endif

#endif  // PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
