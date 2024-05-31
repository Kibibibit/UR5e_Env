// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from par_interfaces:action/GripperSetWidth.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__BUILDER_HPP_
#define PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "par_interfaces/action/detail/gripper_set_width__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_Goal_target_force
{
public:
  explicit Init_GripperSetWidth_Goal_target_force(::par_interfaces::action::GripperSetWidth_Goal & msg)
  : msg_(msg)
  {}
  ::par_interfaces::action::GripperSetWidth_Goal target_force(::par_interfaces::action::GripperSetWidth_Goal::_target_force_type arg)
  {
    msg_.target_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_Goal msg_;
};

class Init_GripperSetWidth_Goal_target_width
{
public:
  Init_GripperSetWidth_Goal_target_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperSetWidth_Goal_target_force target_width(::par_interfaces::action::GripperSetWidth_Goal::_target_width_type arg)
  {
    msg_.target_width = std::move(arg);
    return Init_GripperSetWidth_Goal_target_force(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_Goal>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_Goal_target_width();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_Result_final_width
{
public:
  Init_GripperSetWidth_Result_final_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::par_interfaces::action::GripperSetWidth_Result final_width(::par_interfaces::action::GripperSetWidth_Result::_final_width_type arg)
  {
    msg_.final_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_Result>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_Result_final_width();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_Feedback_current_width
{
public:
  Init_GripperSetWidth_Feedback_current_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::par_interfaces::action::GripperSetWidth_Feedback current_width(::par_interfaces::action::GripperSetWidth_Feedback::_current_width_type arg)
  {
    msg_.current_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_Feedback>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_Feedback_current_width();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_SendGoal_Request_goal
{
public:
  explicit Init_GripperSetWidth_SendGoal_Request_goal(::par_interfaces::action::GripperSetWidth_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::par_interfaces::action::GripperSetWidth_SendGoal_Request goal(::par_interfaces::action::GripperSetWidth_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_SendGoal_Request msg_;
};

class Init_GripperSetWidth_SendGoal_Request_goal_id
{
public:
  Init_GripperSetWidth_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperSetWidth_SendGoal_Request_goal goal_id(::par_interfaces::action::GripperSetWidth_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GripperSetWidth_SendGoal_Request_goal(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_SendGoal_Request>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_SendGoal_Request_goal_id();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_SendGoal_Response_stamp
{
public:
  explicit Init_GripperSetWidth_SendGoal_Response_stamp(::par_interfaces::action::GripperSetWidth_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::par_interfaces::action::GripperSetWidth_SendGoal_Response stamp(::par_interfaces::action::GripperSetWidth_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_SendGoal_Response msg_;
};

class Init_GripperSetWidth_SendGoal_Response_accepted
{
public:
  Init_GripperSetWidth_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperSetWidth_SendGoal_Response_stamp accepted(::par_interfaces::action::GripperSetWidth_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GripperSetWidth_SendGoal_Response_stamp(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_SendGoal_Response>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_SendGoal_Response_accepted();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_GetResult_Request_goal_id
{
public:
  Init_GripperSetWidth_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::par_interfaces::action::GripperSetWidth_GetResult_Request goal_id(::par_interfaces::action::GripperSetWidth_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_GetResult_Request>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_GetResult_Request_goal_id();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_GetResult_Response_result
{
public:
  explicit Init_GripperSetWidth_GetResult_Response_result(::par_interfaces::action::GripperSetWidth_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::par_interfaces::action::GripperSetWidth_GetResult_Response result(::par_interfaces::action::GripperSetWidth_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_GetResult_Response msg_;
};

class Init_GripperSetWidth_GetResult_Response_status
{
public:
  Init_GripperSetWidth_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperSetWidth_GetResult_Response_result status(::par_interfaces::action::GripperSetWidth_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GripperSetWidth_GetResult_Response_result(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_GetResult_Response>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_GetResult_Response_status();
}

}  // namespace par_interfaces


namespace par_interfaces
{

namespace action
{

namespace builder
{

class Init_GripperSetWidth_FeedbackMessage_feedback
{
public:
  explicit Init_GripperSetWidth_FeedbackMessage_feedback(::par_interfaces::action::GripperSetWidth_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::par_interfaces::action::GripperSetWidth_FeedbackMessage feedback(::par_interfaces::action::GripperSetWidth_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_FeedbackMessage msg_;
};

class Init_GripperSetWidth_FeedbackMessage_goal_id
{
public:
  Init_GripperSetWidth_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperSetWidth_FeedbackMessage_feedback goal_id(::par_interfaces::action::GripperSetWidth_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GripperSetWidth_FeedbackMessage_feedback(msg_);
  }

private:
  ::par_interfaces::action::GripperSetWidth_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::par_interfaces::action::GripperSetWidth_FeedbackMessage>()
{
  return par_interfaces::action::builder::Init_GripperSetWidth_FeedbackMessage_goal_id();
}

}  // namespace par_interfaces

#endif  // PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__BUILDER_HPP_
