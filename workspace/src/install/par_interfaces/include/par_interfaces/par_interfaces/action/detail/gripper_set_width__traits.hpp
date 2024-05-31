// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from par_interfaces:action/GripperSetWidth.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__TRAITS_HPP_
#define PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "par_interfaces/action/detail/gripper_set_width__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_width
  {
    out << "target_width: ";
    rosidl_generator_traits::value_to_yaml(msg.target_width, out);
    out << ", ";
  }

  // member: target_force
  {
    out << "target_force: ";
    rosidl_generator_traits::value_to_yaml(msg.target_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_width: ";
    rosidl_generator_traits::value_to_yaml(msg.target_width, out);
    out << "\n";
  }

  // member: target_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_force: ";
    rosidl_generator_traits::value_to_yaml(msg.target_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_Goal & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_Goal>()
{
  return "par_interfaces::action::GripperSetWidth_Goal";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_Goal>()
{
  return "par_interfaces/action/GripperSetWidth_Goal";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: final_width
  {
    out << "final_width: ";
    rosidl_generator_traits::value_to_yaml(msg.final_width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: final_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_width: ";
    rosidl_generator_traits::value_to_yaml(msg.final_width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_Result & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_Result>()
{
  return "par_interfaces::action::GripperSetWidth_Result";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_Result>()
{
  return "par_interfaces/action/GripperSetWidth_Result";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_width
  {
    out << "current_width: ";
    rosidl_generator_traits::value_to_yaml(msg.current_width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_width: ";
    rosidl_generator_traits::value_to_yaml(msg.current_width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_Feedback & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_Feedback>()
{
  return "par_interfaces::action::GripperSetWidth_Feedback";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_Feedback>()
{
  return "par_interfaces/action/GripperSetWidth_Feedback";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "par_interfaces/action/detail/gripper_set_width__traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_SendGoal_Request & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_SendGoal_Request>()
{
  return "par_interfaces::action::GripperSetWidth_SendGoal_Request";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_SendGoal_Request>()
{
  return "par_interfaces/action/GripperSetWidth_SendGoal_Request";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<par_interfaces::action::GripperSetWidth_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<par_interfaces::action::GripperSetWidth_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_SendGoal_Response & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_SendGoal_Response>()
{
  return "par_interfaces::action::GripperSetWidth_SendGoal_Response";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_SendGoal_Response>()
{
  return "par_interfaces/action/GripperSetWidth_SendGoal_Response";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_SendGoal>()
{
  return "par_interfaces::action::GripperSetWidth_SendGoal";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_SendGoal>()
{
  return "par_interfaces/action/GripperSetWidth_SendGoal";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<par_interfaces::action::GripperSetWidth_SendGoal_Request>::value &&
    has_fixed_size<par_interfaces::action::GripperSetWidth_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<par_interfaces::action::GripperSetWidth_SendGoal_Request>::value &&
    has_bounded_size<par_interfaces::action::GripperSetWidth_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<par_interfaces::action::GripperSetWidth_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<par_interfaces::action::GripperSetWidth_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<par_interfaces::action::GripperSetWidth_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_GetResult_Request & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_GetResult_Request>()
{
  return "par_interfaces::action::GripperSetWidth_GetResult_Request";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_GetResult_Request>()
{
  return "par_interfaces/action/GripperSetWidth_GetResult_Request";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_GetResult_Response & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_GetResult_Response>()
{
  return "par_interfaces::action::GripperSetWidth_GetResult_Response";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_GetResult_Response>()
{
  return "par_interfaces/action/GripperSetWidth_GetResult_Response";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<par_interfaces::action::GripperSetWidth_Result>::value> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<par_interfaces::action::GripperSetWidth_Result>::value> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_GetResult>()
{
  return "par_interfaces::action::GripperSetWidth_GetResult";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_GetResult>()
{
  return "par_interfaces/action/GripperSetWidth_GetResult";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<par_interfaces::action::GripperSetWidth_GetResult_Request>::value &&
    has_fixed_size<par_interfaces::action::GripperSetWidth_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<par_interfaces::action::GripperSetWidth_GetResult_Request>::value &&
    has_bounded_size<par_interfaces::action::GripperSetWidth_GetResult_Response>::value
  >
{
};

template<>
struct is_service<par_interfaces::action::GripperSetWidth_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<par_interfaces::action::GripperSetWidth_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<par_interfaces::action::GripperSetWidth_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__traits.hpp"

namespace par_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const GripperSetWidth_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperSetWidth_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperSetWidth_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace par_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use par_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const par_interfaces::action::GripperSetWidth_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  par_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use par_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const par_interfaces::action::GripperSetWidth_FeedbackMessage & msg)
{
  return par_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<par_interfaces::action::GripperSetWidth_FeedbackMessage>()
{
  return "par_interfaces::action::GripperSetWidth_FeedbackMessage";
}

template<>
inline const char * name<par_interfaces::action::GripperSetWidth_FeedbackMessage>()
{
  return "par_interfaces/action/GripperSetWidth_FeedbackMessage";
}

template<>
struct has_fixed_size<par_interfaces::action::GripperSetWidth_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<par_interfaces::action::GripperSetWidth_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<par_interfaces::action::GripperSetWidth_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<par_interfaces::action::GripperSetWidth_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<par_interfaces::action::GripperSetWidth_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<par_interfaces::action::GripperSetWidth>
  : std::true_type
{
};

template<>
struct is_action_goal<par_interfaces::action::GripperSetWidth_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<par_interfaces::action::GripperSetWidth_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<par_interfaces::action::GripperSetWidth_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__TRAITS_HPP_
