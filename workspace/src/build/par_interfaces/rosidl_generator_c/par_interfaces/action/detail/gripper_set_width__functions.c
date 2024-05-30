// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from par_interfaces:action/GripperSetWidth.idl
// generated code does not contain a copyright notice
#include "par_interfaces/action/detail/gripper_set_width__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
par_interfaces__action__GripperSetWidth_Goal__init(par_interfaces__action__GripperSetWidth_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_width
  // target_force
  return true;
}

void
par_interfaces__action__GripperSetWidth_Goal__fini(par_interfaces__action__GripperSetWidth_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_width
  // target_force
}

bool
par_interfaces__action__GripperSetWidth_Goal__are_equal(const par_interfaces__action__GripperSetWidth_Goal * lhs, const par_interfaces__action__GripperSetWidth_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_width
  if (lhs->target_width != rhs->target_width) {
    return false;
  }
  // target_force
  if (lhs->target_force != rhs->target_force) {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Goal__copy(
  const par_interfaces__action__GripperSetWidth_Goal * input,
  par_interfaces__action__GripperSetWidth_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_width
  output->target_width = input->target_width;
  // target_force
  output->target_force = input->target_force;
  return true;
}

par_interfaces__action__GripperSetWidth_Goal *
par_interfaces__action__GripperSetWidth_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Goal * msg = (par_interfaces__action__GripperSetWidth_Goal *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_Goal));
  bool success = par_interfaces__action__GripperSetWidth_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_Goal__destroy(par_interfaces__action__GripperSetWidth_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_Goal__Sequence__init(par_interfaces__action__GripperSetWidth_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Goal * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_Goal *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_Goal__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_Goal__Sequence__fini(par_interfaces__action__GripperSetWidth_Goal__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_Goal__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_Goal__Sequence *
par_interfaces__action__GripperSetWidth_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Goal__Sequence * array = (par_interfaces__action__GripperSetWidth_Goal__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_Goal__Sequence__destroy(par_interfaces__action__GripperSetWidth_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_Goal__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_Goal__Sequence * lhs, const par_interfaces__action__GripperSetWidth_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Goal__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_Goal__Sequence * input,
  par_interfaces__action__GripperSetWidth_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_Goal * data =
      (par_interfaces__action__GripperSetWidth_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
par_interfaces__action__GripperSetWidth_Result__init(par_interfaces__action__GripperSetWidth_Result * msg)
{
  if (!msg) {
    return false;
  }
  // final_width
  return true;
}

void
par_interfaces__action__GripperSetWidth_Result__fini(par_interfaces__action__GripperSetWidth_Result * msg)
{
  if (!msg) {
    return;
  }
  // final_width
}

bool
par_interfaces__action__GripperSetWidth_Result__are_equal(const par_interfaces__action__GripperSetWidth_Result * lhs, const par_interfaces__action__GripperSetWidth_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // final_width
  if (lhs->final_width != rhs->final_width) {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Result__copy(
  const par_interfaces__action__GripperSetWidth_Result * input,
  par_interfaces__action__GripperSetWidth_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // final_width
  output->final_width = input->final_width;
  return true;
}

par_interfaces__action__GripperSetWidth_Result *
par_interfaces__action__GripperSetWidth_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Result * msg = (par_interfaces__action__GripperSetWidth_Result *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_Result));
  bool success = par_interfaces__action__GripperSetWidth_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_Result__destroy(par_interfaces__action__GripperSetWidth_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_Result__Sequence__init(par_interfaces__action__GripperSetWidth_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Result * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_Result *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_Result__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_Result__Sequence__fini(par_interfaces__action__GripperSetWidth_Result__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_Result__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_Result__Sequence *
par_interfaces__action__GripperSetWidth_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Result__Sequence * array = (par_interfaces__action__GripperSetWidth_Result__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_Result__Sequence__destroy(par_interfaces__action__GripperSetWidth_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_Result__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_Result__Sequence * lhs, const par_interfaces__action__GripperSetWidth_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Result__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_Result__Sequence * input,
  par_interfaces__action__GripperSetWidth_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_Result * data =
      (par_interfaces__action__GripperSetWidth_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
par_interfaces__action__GripperSetWidth_Feedback__init(par_interfaces__action__GripperSetWidth_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_width
  return true;
}

void
par_interfaces__action__GripperSetWidth_Feedback__fini(par_interfaces__action__GripperSetWidth_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_width
}

bool
par_interfaces__action__GripperSetWidth_Feedback__are_equal(const par_interfaces__action__GripperSetWidth_Feedback * lhs, const par_interfaces__action__GripperSetWidth_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_width
  if (lhs->current_width != rhs->current_width) {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Feedback__copy(
  const par_interfaces__action__GripperSetWidth_Feedback * input,
  par_interfaces__action__GripperSetWidth_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_width
  output->current_width = input->current_width;
  return true;
}

par_interfaces__action__GripperSetWidth_Feedback *
par_interfaces__action__GripperSetWidth_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Feedback * msg = (par_interfaces__action__GripperSetWidth_Feedback *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_Feedback));
  bool success = par_interfaces__action__GripperSetWidth_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_Feedback__destroy(par_interfaces__action__GripperSetWidth_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_Feedback__Sequence__init(par_interfaces__action__GripperSetWidth_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Feedback * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_Feedback *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_Feedback__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_Feedback__Sequence__fini(par_interfaces__action__GripperSetWidth_Feedback__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_Feedback__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_Feedback__Sequence *
par_interfaces__action__GripperSetWidth_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_Feedback__Sequence * array = (par_interfaces__action__GripperSetWidth_Feedback__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_Feedback__Sequence__destroy(par_interfaces__action__GripperSetWidth_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_Feedback__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_Feedback__Sequence * lhs, const par_interfaces__action__GripperSetWidth_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_Feedback__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_Feedback__Sequence * input,
  par_interfaces__action__GripperSetWidth_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_Feedback * data =
      (par_interfaces__action__GripperSetWidth_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__functions.h"

bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__init(par_interfaces__action__GripperSetWidth_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!par_interfaces__action__GripperSetWidth_Goal__init(&msg->goal)) {
    par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(par_interfaces__action__GripperSetWidth_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  par_interfaces__action__GripperSetWidth_Goal__fini(&msg->goal);
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__are_equal(const par_interfaces__action__GripperSetWidth_SendGoal_Request * lhs, const par_interfaces__action__GripperSetWidth_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!par_interfaces__action__GripperSetWidth_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__copy(
  const par_interfaces__action__GripperSetWidth_SendGoal_Request * input,
  par_interfaces__action__GripperSetWidth_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!par_interfaces__action__GripperSetWidth_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

par_interfaces__action__GripperSetWidth_SendGoal_Request *
par_interfaces__action__GripperSetWidth_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Request * msg = (par_interfaces__action__GripperSetWidth_SendGoal_Request *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Request));
  bool success = par_interfaces__action__GripperSetWidth_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Request__destroy(par_interfaces__action__GripperSetWidth_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__init(par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Request * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_SendGoal_Request *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__fini(par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence *
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * array = (par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__destroy(par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * lhs, const par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * input,
  par_interfaces__action__GripperSetWidth_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_SendGoal_Request * data =
      (par_interfaces__action__GripperSetWidth_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__init(par_interfaces__action__GripperSetWidth_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(par_interfaces__action__GripperSetWidth_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__are_equal(const par_interfaces__action__GripperSetWidth_SendGoal_Response * lhs, const par_interfaces__action__GripperSetWidth_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__copy(
  const par_interfaces__action__GripperSetWidth_SendGoal_Response * input,
  par_interfaces__action__GripperSetWidth_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

par_interfaces__action__GripperSetWidth_SendGoal_Response *
par_interfaces__action__GripperSetWidth_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Response * msg = (par_interfaces__action__GripperSetWidth_SendGoal_Response *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Response));
  bool success = par_interfaces__action__GripperSetWidth_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Response__destroy(par_interfaces__action__GripperSetWidth_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__init(par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Response * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_SendGoal_Response *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__fini(par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence *
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * array = (par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__destroy(par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * lhs, const par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * input,
  par_interfaces__action__GripperSetWidth_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_SendGoal_Response * data =
      (par_interfaces__action__GripperSetWidth_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
par_interfaces__action__GripperSetWidth_GetResult_Request__init(par_interfaces__action__GripperSetWidth_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    par_interfaces__action__GripperSetWidth_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Request__fini(par_interfaces__action__GripperSetWidth_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Request__are_equal(const par_interfaces__action__GripperSetWidth_GetResult_Request * lhs, const par_interfaces__action__GripperSetWidth_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Request__copy(
  const par_interfaces__action__GripperSetWidth_GetResult_Request * input,
  par_interfaces__action__GripperSetWidth_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

par_interfaces__action__GripperSetWidth_GetResult_Request *
par_interfaces__action__GripperSetWidth_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Request * msg = (par_interfaces__action__GripperSetWidth_GetResult_Request *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_GetResult_Request));
  bool success = par_interfaces__action__GripperSetWidth_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Request__destroy(par_interfaces__action__GripperSetWidth_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__init(par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Request * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_GetResult_Request *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_GetResult_Request__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__fini(par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_GetResult_Request__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence *
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * array = (par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__destroy(par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * lhs, const par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * input,
  par_interfaces__action__GripperSetWidth_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_GetResult_Request * data =
      (par_interfaces__action__GripperSetWidth_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__functions.h"

bool
par_interfaces__action__GripperSetWidth_GetResult_Response__init(par_interfaces__action__GripperSetWidth_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!par_interfaces__action__GripperSetWidth_Result__init(&msg->result)) {
    par_interfaces__action__GripperSetWidth_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Response__fini(par_interfaces__action__GripperSetWidth_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  par_interfaces__action__GripperSetWidth_Result__fini(&msg->result);
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Response__are_equal(const par_interfaces__action__GripperSetWidth_GetResult_Response * lhs, const par_interfaces__action__GripperSetWidth_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!par_interfaces__action__GripperSetWidth_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Response__copy(
  const par_interfaces__action__GripperSetWidth_GetResult_Response * input,
  par_interfaces__action__GripperSetWidth_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!par_interfaces__action__GripperSetWidth_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

par_interfaces__action__GripperSetWidth_GetResult_Response *
par_interfaces__action__GripperSetWidth_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Response * msg = (par_interfaces__action__GripperSetWidth_GetResult_Response *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_GetResult_Response));
  bool success = par_interfaces__action__GripperSetWidth_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Response__destroy(par_interfaces__action__GripperSetWidth_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__init(par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Response * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_GetResult_Response *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_GetResult_Response__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__fini(par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_GetResult_Response__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence *
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * array = (par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__destroy(par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * lhs, const par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * input,
  par_interfaces__action__GripperSetWidth_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_GetResult_Response * data =
      (par_interfaces__action__GripperSetWidth_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "par_interfaces/action/detail/gripper_set_width__functions.h"

bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__init(par_interfaces__action__GripperSetWidth_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!par_interfaces__action__GripperSetWidth_Feedback__init(&msg->feedback)) {
    par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(par_interfaces__action__GripperSetWidth_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  par_interfaces__action__GripperSetWidth_Feedback__fini(&msg->feedback);
}

bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__are_equal(const par_interfaces__action__GripperSetWidth_FeedbackMessage * lhs, const par_interfaces__action__GripperSetWidth_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!par_interfaces__action__GripperSetWidth_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__copy(
  const par_interfaces__action__GripperSetWidth_FeedbackMessage * input,
  par_interfaces__action__GripperSetWidth_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!par_interfaces__action__GripperSetWidth_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

par_interfaces__action__GripperSetWidth_FeedbackMessage *
par_interfaces__action__GripperSetWidth_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_FeedbackMessage * msg = (par_interfaces__action__GripperSetWidth_FeedbackMessage *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(par_interfaces__action__GripperSetWidth_FeedbackMessage));
  bool success = par_interfaces__action__GripperSetWidth_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
par_interfaces__action__GripperSetWidth_FeedbackMessage__destroy(par_interfaces__action__GripperSetWidth_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__init(par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_FeedbackMessage * data = NULL;

  if (size) {
    data = (par_interfaces__action__GripperSetWidth_FeedbackMessage *)allocator.zero_allocate(size, sizeof(par_interfaces__action__GripperSetWidth_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = par_interfaces__action__GripperSetWidth_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(&data[i - 1]);
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
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__fini(par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * array)
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
      par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(&array->data[i]);
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

par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence *
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * array = (par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence *)allocator.allocate(sizeof(par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__destroy(par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__are_equal(const par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * lhs, const par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence__copy(
  const par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * input,
  par_interfaces__action__GripperSetWidth_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(par_interfaces__action__GripperSetWidth_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    par_interfaces__action__GripperSetWidth_FeedbackMessage * data =
      (par_interfaces__action__GripperSetWidth_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!par_interfaces__action__GripperSetWidth_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          par_interfaces__action__GripperSetWidth_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!par_interfaces__action__GripperSetWidth_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
