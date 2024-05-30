// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from par_interfaces:action/GripperSetWidth.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__STRUCT_H_
#define PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_Goal
{
  /// Request Params:
  /// The target width of the gripper in metres.
  /// If the request is outside the valid range, this will get clamped
  int32_t target_width;
  /// The target force to exert in newtons. If outside the valid range, this will get clamped
  int32_t target_force;
} par_interfaces__action__GripperSetWidth_Goal;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_Goal.
typedef struct par_interfaces__action__GripperSetWidth_Goal__Sequence
{
  par_interfaces__action__GripperSetWidth_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_Result
{
  int32_t final_width;
} par_interfaces__action__GripperSetWidth_Result;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_Result.
typedef struct par_interfaces__action__GripperSetWidth_Result__Sequence
{
  par_interfaces__action__GripperSetWidth_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_Feedback
{
  int32_t current_width;
} par_interfaces__action__GripperSetWidth_Feedback;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_Feedback.
typedef struct par_interfaces__action__GripperSetWidth_Feedback__Sequence
{
  par_interfaces__action__GripperSetWidth_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "par_interfaces/action/detail/gripper_set_width__struct.h"

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  par_interfaces__action__GripperSetWidth_Goal goal;
} par_interfaces__action__GripperSetWidth_SendGoal_Request;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_SendGoal_Request.
typedef struct par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence
{
  par_interfaces__action__GripperSetWidth_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} par_interfaces__action__GripperSetWidth_SendGoal_Response;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_SendGoal_Response.
typedef struct par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence
{
  par_interfaces__action__GripperSetWidth_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} par_interfaces__action__GripperSetWidth_GetResult_Request;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_GetResult_Request.
typedef struct par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence
{
  par_interfaces__action__GripperSetWidth_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__struct.h"

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_GetResult_Response
{
  int8_t status;
  par_interfaces__action__GripperSetWidth_Result result;
} par_interfaces__action__GripperSetWidth_GetResult_Response;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_GetResult_Response.
typedef struct par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence
{
  par_interfaces__action__GripperSetWidth_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__struct.h"

/// Struct defined in action/GripperSetWidth in the package par_interfaces.
typedef struct par_interfaces__action__GripperSetWidth_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  par_interfaces__action__GripperSetWidth_Feedback feedback;
} par_interfaces__action__GripperSetWidth_FeedbackMessage;

// Struct for a sequence of par_interfaces__action__GripperSetWidth_FeedbackMessage.
typedef struct par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence
{
  par_interfaces__action__GripperSetWidth_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PAR_INTERFACES__ACTION__DETAIL__GRIPPER_SET_WIDTH__STRUCT_H_
