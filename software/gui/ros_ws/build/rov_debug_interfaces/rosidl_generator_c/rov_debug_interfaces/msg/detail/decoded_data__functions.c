// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice
#include "rov_debug_interfaces/msg/detail/decoded_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `imu`
#include "rov_debug_interfaces/msg/detail/imu__functions.h"

bool
rov_debug_interfaces__msg__DecodedData__init(rov_debug_interfaces__msg__DecodedData * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // valid
  // imu
  if (!rov_debug_interfaces__msg__Imu__init(&msg->imu)) {
    rov_debug_interfaces__msg__DecodedData__fini(msg);
    return false;
  }
  // thruster_current_1
  // thruster_current_2
  // thruster_current_3
  // thruster_current_4
  // thruster_current_5
  // thruster_current_6
  // thruster_pwm_1
  // thruster_pwm_2
  // thruster_pwm_3
  // thruster_pwm_4
  // thruster_pwm_5
  // thruster_pwm_6
  // indicator_1
  // indicator_2
  // indicator_3
  // indicator_4
  // indicator_5
  // indicator_6
  // heartbeat_1
  // heartbeat_2
  // heartbeat_3
  // heartbeat_4
  // connection_percentage_1
  // connection_percentage_2
  // connection_percentage_3
  // connection_percentage_4
  // arm_1
  // arm_2
  // arm_3
  // arm_4
  // rov_depth
  return true;
}

void
rov_debug_interfaces__msg__DecodedData__fini(rov_debug_interfaces__msg__DecodedData * msg)
{
  if (!msg) {
    return;
  }
  // id
  // valid
  // imu
  rov_debug_interfaces__msg__Imu__fini(&msg->imu);
  // thruster_current_1
  // thruster_current_2
  // thruster_current_3
  // thruster_current_4
  // thruster_current_5
  // thruster_current_6
  // thruster_pwm_1
  // thruster_pwm_2
  // thruster_pwm_3
  // thruster_pwm_4
  // thruster_pwm_5
  // thruster_pwm_6
  // indicator_1
  // indicator_2
  // indicator_3
  // indicator_4
  // indicator_5
  // indicator_6
  // heartbeat_1
  // heartbeat_2
  // heartbeat_3
  // heartbeat_4
  // connection_percentage_1
  // connection_percentage_2
  // connection_percentage_3
  // connection_percentage_4
  // arm_1
  // arm_2
  // arm_3
  // arm_4
  // rov_depth
}

bool
rov_debug_interfaces__msg__DecodedData__are_equal(const rov_debug_interfaces__msg__DecodedData * lhs, const rov_debug_interfaces__msg__DecodedData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // imu
  if (!rov_debug_interfaces__msg__Imu__are_equal(
      &(lhs->imu), &(rhs->imu)))
  {
    return false;
  }
  // thruster_current_1
  if (lhs->thruster_current_1 != rhs->thruster_current_1) {
    return false;
  }
  // thruster_current_2
  if (lhs->thruster_current_2 != rhs->thruster_current_2) {
    return false;
  }
  // thruster_current_3
  if (lhs->thruster_current_3 != rhs->thruster_current_3) {
    return false;
  }
  // thruster_current_4
  if (lhs->thruster_current_4 != rhs->thruster_current_4) {
    return false;
  }
  // thruster_current_5
  if (lhs->thruster_current_5 != rhs->thruster_current_5) {
    return false;
  }
  // thruster_current_6
  if (lhs->thruster_current_6 != rhs->thruster_current_6) {
    return false;
  }
  // thruster_pwm_1
  if (lhs->thruster_pwm_1 != rhs->thruster_pwm_1) {
    return false;
  }
  // thruster_pwm_2
  if (lhs->thruster_pwm_2 != rhs->thruster_pwm_2) {
    return false;
  }
  // thruster_pwm_3
  if (lhs->thruster_pwm_3 != rhs->thruster_pwm_3) {
    return false;
  }
  // thruster_pwm_4
  if (lhs->thruster_pwm_4 != rhs->thruster_pwm_4) {
    return false;
  }
  // thruster_pwm_5
  if (lhs->thruster_pwm_5 != rhs->thruster_pwm_5) {
    return false;
  }
  // thruster_pwm_6
  if (lhs->thruster_pwm_6 != rhs->thruster_pwm_6) {
    return false;
  }
  // indicator_1
  if (lhs->indicator_1 != rhs->indicator_1) {
    return false;
  }
  // indicator_2
  if (lhs->indicator_2 != rhs->indicator_2) {
    return false;
  }
  // indicator_3
  if (lhs->indicator_3 != rhs->indicator_3) {
    return false;
  }
  // indicator_4
  if (lhs->indicator_4 != rhs->indicator_4) {
    return false;
  }
  // indicator_5
  if (lhs->indicator_5 != rhs->indicator_5) {
    return false;
  }
  // indicator_6
  if (lhs->indicator_6 != rhs->indicator_6) {
    return false;
  }
  // heartbeat_1
  if (lhs->heartbeat_1 != rhs->heartbeat_1) {
    return false;
  }
  // heartbeat_2
  if (lhs->heartbeat_2 != rhs->heartbeat_2) {
    return false;
  }
  // heartbeat_3
  if (lhs->heartbeat_3 != rhs->heartbeat_3) {
    return false;
  }
  // heartbeat_4
  if (lhs->heartbeat_4 != rhs->heartbeat_4) {
    return false;
  }
  // connection_percentage_1
  if (lhs->connection_percentage_1 != rhs->connection_percentage_1) {
    return false;
  }
  // connection_percentage_2
  if (lhs->connection_percentage_2 != rhs->connection_percentage_2) {
    return false;
  }
  // connection_percentage_3
  if (lhs->connection_percentage_3 != rhs->connection_percentage_3) {
    return false;
  }
  // connection_percentage_4
  if (lhs->connection_percentage_4 != rhs->connection_percentage_4) {
    return false;
  }
  // arm_1
  if (lhs->arm_1 != rhs->arm_1) {
    return false;
  }
  // arm_2
  if (lhs->arm_2 != rhs->arm_2) {
    return false;
  }
  // arm_3
  if (lhs->arm_3 != rhs->arm_3) {
    return false;
  }
  // arm_4
  if (lhs->arm_4 != rhs->arm_4) {
    return false;
  }
  // rov_depth
  if (lhs->rov_depth != rhs->rov_depth) {
    return false;
  }
  return true;
}

bool
rov_debug_interfaces__msg__DecodedData__copy(
  const rov_debug_interfaces__msg__DecodedData * input,
  rov_debug_interfaces__msg__DecodedData * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // valid
  output->valid = input->valid;
  // imu
  if (!rov_debug_interfaces__msg__Imu__copy(
      &(input->imu), &(output->imu)))
  {
    return false;
  }
  // thruster_current_1
  output->thruster_current_1 = input->thruster_current_1;
  // thruster_current_2
  output->thruster_current_2 = input->thruster_current_2;
  // thruster_current_3
  output->thruster_current_3 = input->thruster_current_3;
  // thruster_current_4
  output->thruster_current_4 = input->thruster_current_4;
  // thruster_current_5
  output->thruster_current_5 = input->thruster_current_5;
  // thruster_current_6
  output->thruster_current_6 = input->thruster_current_6;
  // thruster_pwm_1
  output->thruster_pwm_1 = input->thruster_pwm_1;
  // thruster_pwm_2
  output->thruster_pwm_2 = input->thruster_pwm_2;
  // thruster_pwm_3
  output->thruster_pwm_3 = input->thruster_pwm_3;
  // thruster_pwm_4
  output->thruster_pwm_4 = input->thruster_pwm_4;
  // thruster_pwm_5
  output->thruster_pwm_5 = input->thruster_pwm_5;
  // thruster_pwm_6
  output->thruster_pwm_6 = input->thruster_pwm_6;
  // indicator_1
  output->indicator_1 = input->indicator_1;
  // indicator_2
  output->indicator_2 = input->indicator_2;
  // indicator_3
  output->indicator_3 = input->indicator_3;
  // indicator_4
  output->indicator_4 = input->indicator_4;
  // indicator_5
  output->indicator_5 = input->indicator_5;
  // indicator_6
  output->indicator_6 = input->indicator_6;
  // heartbeat_1
  output->heartbeat_1 = input->heartbeat_1;
  // heartbeat_2
  output->heartbeat_2 = input->heartbeat_2;
  // heartbeat_3
  output->heartbeat_3 = input->heartbeat_3;
  // heartbeat_4
  output->heartbeat_4 = input->heartbeat_4;
  // connection_percentage_1
  output->connection_percentage_1 = input->connection_percentage_1;
  // connection_percentage_2
  output->connection_percentage_2 = input->connection_percentage_2;
  // connection_percentage_3
  output->connection_percentage_3 = input->connection_percentage_3;
  // connection_percentage_4
  output->connection_percentage_4 = input->connection_percentage_4;
  // arm_1
  output->arm_1 = input->arm_1;
  // arm_2
  output->arm_2 = input->arm_2;
  // arm_3
  output->arm_3 = input->arm_3;
  // arm_4
  output->arm_4 = input->arm_4;
  // rov_depth
  output->rov_depth = input->rov_depth;
  return true;
}

rov_debug_interfaces__msg__DecodedData *
rov_debug_interfaces__msg__DecodedData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__DecodedData * msg = (rov_debug_interfaces__msg__DecodedData *)allocator.allocate(sizeof(rov_debug_interfaces__msg__DecodedData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rov_debug_interfaces__msg__DecodedData));
  bool success = rov_debug_interfaces__msg__DecodedData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rov_debug_interfaces__msg__DecodedData__destroy(rov_debug_interfaces__msg__DecodedData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rov_debug_interfaces__msg__DecodedData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rov_debug_interfaces__msg__DecodedData__Sequence__init(rov_debug_interfaces__msg__DecodedData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__DecodedData * data = NULL;

  if (size) {
    data = (rov_debug_interfaces__msg__DecodedData *)allocator.zero_allocate(size, sizeof(rov_debug_interfaces__msg__DecodedData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rov_debug_interfaces__msg__DecodedData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rov_debug_interfaces__msg__DecodedData__fini(&data[i - 1]);
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
rov_debug_interfaces__msg__DecodedData__Sequence__fini(rov_debug_interfaces__msg__DecodedData__Sequence * array)
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
      rov_debug_interfaces__msg__DecodedData__fini(&array->data[i]);
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

rov_debug_interfaces__msg__DecodedData__Sequence *
rov_debug_interfaces__msg__DecodedData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__DecodedData__Sequence * array = (rov_debug_interfaces__msg__DecodedData__Sequence *)allocator.allocate(sizeof(rov_debug_interfaces__msg__DecodedData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rov_debug_interfaces__msg__DecodedData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rov_debug_interfaces__msg__DecodedData__Sequence__destroy(rov_debug_interfaces__msg__DecodedData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rov_debug_interfaces__msg__DecodedData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rov_debug_interfaces__msg__DecodedData__Sequence__are_equal(const rov_debug_interfaces__msg__DecodedData__Sequence * lhs, const rov_debug_interfaces__msg__DecodedData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rov_debug_interfaces__msg__DecodedData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rov_debug_interfaces__msg__DecodedData__Sequence__copy(
  const rov_debug_interfaces__msg__DecodedData__Sequence * input,
  rov_debug_interfaces__msg__DecodedData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rov_debug_interfaces__msg__DecodedData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rov_debug_interfaces__msg__DecodedData * data =
      (rov_debug_interfaces__msg__DecodedData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rov_debug_interfaces__msg__DecodedData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rov_debug_interfaces__msg__DecodedData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rov_debug_interfaces__msg__DecodedData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
