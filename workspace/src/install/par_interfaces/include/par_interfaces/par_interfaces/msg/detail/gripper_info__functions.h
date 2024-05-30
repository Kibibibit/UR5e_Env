// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__FUNCTIONS_H_
#define PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "par_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "par_interfaces/msg/detail/gripper_info__struct.h"

/// Initialize msg/GripperInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * par_interfaces__msg__GripperInfo
 * )) before or use
 * par_interfaces__msg__GripperInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__init(par_interfaces__msg__GripperInfo * msg);

/// Finalize msg/GripperInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
void
par_interfaces__msg__GripperInfo__fini(par_interfaces__msg__GripperInfo * msg);

/// Create msg/GripperInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * par_interfaces__msg__GripperInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
par_interfaces__msg__GripperInfo *
par_interfaces__msg__GripperInfo__create();

/// Destroy msg/GripperInfo message.
/**
 * It calls
 * par_interfaces__msg__GripperInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
void
par_interfaces__msg__GripperInfo__destroy(par_interfaces__msg__GripperInfo * msg);

/// Check for msg/GripperInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__are_equal(const par_interfaces__msg__GripperInfo * lhs, const par_interfaces__msg__GripperInfo * rhs);

/// Copy a msg/GripperInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__copy(
  const par_interfaces__msg__GripperInfo * input,
  par_interfaces__msg__GripperInfo * output);

/// Initialize array of msg/GripperInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * par_interfaces__msg__GripperInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__Sequence__init(par_interfaces__msg__GripperInfo__Sequence * array, size_t size);

/// Finalize array of msg/GripperInfo messages.
/**
 * It calls
 * par_interfaces__msg__GripperInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
void
par_interfaces__msg__GripperInfo__Sequence__fini(par_interfaces__msg__GripperInfo__Sequence * array);

/// Create array of msg/GripperInfo messages.
/**
 * It allocates the memory for the array and calls
 * par_interfaces__msg__GripperInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
par_interfaces__msg__GripperInfo__Sequence *
par_interfaces__msg__GripperInfo__Sequence__create(size_t size);

/// Destroy array of msg/GripperInfo messages.
/**
 * It calls
 * par_interfaces__msg__GripperInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
void
par_interfaces__msg__GripperInfo__Sequence__destroy(par_interfaces__msg__GripperInfo__Sequence * array);

/// Check for msg/GripperInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__Sequence__are_equal(const par_interfaces__msg__GripperInfo__Sequence * lhs, const par_interfaces__msg__GripperInfo__Sequence * rhs);

/// Copy an array of msg/GripperInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_par_interfaces
bool
par_interfaces__msg__GripperInfo__Sequence__copy(
  const par_interfaces__msg__GripperInfo__Sequence * input,
  par_interfaces__msg__GripperInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__FUNCTIONS_H_
