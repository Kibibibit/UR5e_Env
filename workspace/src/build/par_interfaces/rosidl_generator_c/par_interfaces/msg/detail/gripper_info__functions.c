// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice
#include "par_interfaces/msg/detail/gripper_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `gripper_type`
// Member `ip`
// Member `port`
#include "rosidl_runtime_c/string_functions.h"

bool
par_interfaces__msg__GripperInfo__init(par_interfaces__msg__GripperInfo * msg)
{
  if (!msg) {
    return false;
  }
  // gripper_type
  if (!rosidl_runtime_c__String__init(&msg->gripper_type)) {
    par_interfaces__msg__GripperInfo__fini(msg);
    return false;
  }
  // ip
  if (!rosidl_runtime_c__String__init(&msg->ip)) {
    par_interfaces__msg__GripperInfo__fini(msg);
    return false;
  }
  // port
  if (!rosidl_runtime_c__String__init(&msg->port)) {
    par_interfaces__msg__GripperInfo__fini(msg);
    return false;
  }
  // max_width
  // max_force
  return true;
}

void
par_interfaces__msg__GripperInfo__fini(par_interfaces__msg__GripperInfo * msg)
{
  if (!msg) {
    return;
  }
  // gripper_type
  rosidl_runtime_c__String__fini(&msg->gripper_type);
  // ip
  rosidl_runtime_c__String__fini(&msg->ip);
  // port
  rosidl_runtime_c__String__fini(&msg->port);
  // max_width
  // max_force
}

bool
par_interfaces__msg__GripperInfo__are_equal(const par_interfaces__msg__GripperInfo * lhs, const par_interfaces__msg__GripperInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // gripper_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->gripper_type), &(rhs->gripper_type)))
  {
    return false;
  }
  // ip
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ip), &(rhs->ip)))
  {
    return false;
  }
  // port
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->port), &(rhs->port)))
  {
    return false;
  }
  // max_width
  if (lhs->max_width != rhs->max_width) {
    return false;
  }
  // max_force
  if (lhs->max_force != rhs->max_force) {
    return false;
  }
  return true;
}

bool
par_interfaces__msg__GripperInfo__copy(
  const par_interfaces__msg__GripperInfo * input,
  par_interfaces__msg__GripperInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // gripper_type
  if (!rosidl_runtime_c__String__copy(
      &(input->gripper_type), &(output->gripper_type)))
  {
    return false;
  }
  // ip
  if (!rosidl_runtime_c__String__copy(
      &(input->ip), &(output->ip)))
  {
    return false;
  }
  // port
  if (!rosidl_runtime_c__String__copy(
      &(input->port), &(output->port)))
  {
    return false;
  }
  // max_width
  output->max_width = input->max_width;
  // max_force
  output->max_force = input->max_force;
  return true;
}

par_interfaces__msg__GripperInfo *
par_interfaces__msg__GripperInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__msg__GripperInfo * msg = (par_interfaces__msg__GripperInfo *)allocator.allocate(sizeof(par_interfaces__msg__GripperInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__msg__GripperInfo));
  bool success = par_interfaces__msg__GripperInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__msg__GripperInfo__destroy(par_interfaces__msg__GripperInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__msg__GripperInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__msg__GripperInfo__Sequence__init(par_interfaces__msg__GripperInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__msg__GripperInfo * data = NULL;

  if (size) {
    data = (par_interfaces__msg__GripperInfo *)allocator.zero_allocate(size, sizeof(par_interfaces__msg__GripperInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__msg__GripperInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__msg__GripperInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
par_interfaces__msg__GripperInfo__Sequence__fini(par_interfaces__msg__GripperInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      par_interfaces__msg__GripperInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

par_interfaces__msg__GripperInfo__Sequence *
par_interfaces__msg__GripperInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__msg__GripperInfo__Sequence * array = (par_interfaces__msg__GripperInfo__Sequence *)allocator.allocate(sizeof(par_interfaces__msg__GripperInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__msg__GripperInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__msg__GripperInfo__Sequence__destroy(par_interfaces__msg__GripperInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__msg__GripperInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__msg__GripperInfo__Sequence__are_equal(const par_interfaces__msg__GripperInfo__Sequence * lhs, const par_interfaces__msg__GripperInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__msg__GripperInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__msg__GripperInfo__Sequence__copy(
  const par_interfaces__msg__GripperInfo__Sequence * input,
  par_interfaces__msg__GripperInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__msg__GripperInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__msg__GripperInfo * data =
      (par_interfaces__msg__GripperInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__msg__GripperInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__msg__GripperInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__msg__GripperInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
