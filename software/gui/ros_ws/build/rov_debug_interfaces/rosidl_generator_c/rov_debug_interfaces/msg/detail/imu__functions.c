// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice
#include "rov_debug_interfaces/msg/detail/imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rov_debug_interfaces__msg__Imu__init(rov_debug_interfaces__msg__Imu * msg)
{
  if (!msg) {
    return false;
  }
  // acc_x
  // acc_y
  // acc_z
  // imu_roll
  // imu_pitch
  // imu_yaw
  return true;
}

void
rov_debug_interfaces__msg__Imu__fini(rov_debug_interfaces__msg__Imu * msg)
{
  if (!msg) {
    return;
  }
  // acc_x
  // acc_y
  // acc_z
  // imu_roll
  // imu_pitch
  // imu_yaw
}

bool
rov_debug_interfaces__msg__Imu__are_equal(const rov_debug_interfaces__msg__Imu * lhs, const rov_debug_interfaces__msg__Imu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // acc_x
  if (lhs->acc_x != rhs->acc_x) {
    return false;
  }
  // acc_y
  if (lhs->acc_y != rhs->acc_y) {
    return false;
  }
  // acc_z
  if (lhs->acc_z != rhs->acc_z) {
    return false;
  }
  // imu_roll
  if (lhs->imu_roll != rhs->imu_roll) {
    return false;
  }
  // imu_pitch
  if (lhs->imu_pitch != rhs->imu_pitch) {
    return false;
  }
  // imu_yaw
  if (lhs->imu_yaw != rhs->imu_yaw) {
    return false;
  }
  return true;
}

bool
rov_debug_interfaces__msg__Imu__copy(
  const rov_debug_interfaces__msg__Imu * input,
  rov_debug_interfaces__msg__Imu * output)
{
  if (!input || !output) {
    return false;
  }
  // acc_x
  output->acc_x = input->acc_x;
  // acc_y
  output->acc_y = input->acc_y;
  // acc_z
  output->acc_z = input->acc_z;
  // imu_roll
  output->imu_roll = input->imu_roll;
  // imu_pitch
  output->imu_pitch = input->imu_pitch;
  // imu_yaw
  output->imu_yaw = input->imu_yaw;
  return true;
}

rov_debug_interfaces__msg__Imu *
rov_debug_interfaces__msg__Imu__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__Imu * msg = (rov_debug_interfaces__msg__Imu *)allocator.allocate(sizeof(rov_debug_interfaces__msg__Imu), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rov_debug_interfaces__msg__Imu));
  bool success = rov_debug_interfaces__msg__Imu__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rov_debug_interfaces__msg__Imu__destroy(rov_debug_interfaces__msg__Imu * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rov_debug_interfaces__msg__Imu__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rov_debug_interfaces__msg__Imu__Sequence__init(rov_debug_interfaces__msg__Imu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__Imu * data = NULL;

  if (size) {
    data = (rov_debug_interfaces__msg__Imu *)allocator.zero_allocate(size, sizeof(rov_debug_interfaces__msg__Imu), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rov_debug_interfaces__msg__Imu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rov_debug_interfaces__msg__Imu__fini(&data[i - 1]);
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
rov_debug_interfaces__msg__Imu__Sequence__fini(rov_debug_interfaces__msg__Imu__Sequence * array)
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
      rov_debug_interfaces__msg__Imu__fini(&array->data[i]);
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

rov_debug_interfaces__msg__Imu__Sequence *
rov_debug_interfaces__msg__Imu__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rov_debug_interfaces__msg__Imu__Sequence * array = (rov_debug_interfaces__msg__Imu__Sequence *)allocator.allocate(sizeof(rov_debug_interfaces__msg__Imu__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rov_debug_interfaces__msg__Imu__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rov_debug_interfaces__msg__Imu__Sequence__destroy(rov_debug_interfaces__msg__Imu__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rov_debug_interfaces__msg__Imu__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rov_debug_interfaces__msg__Imu__Sequence__are_equal(const rov_debug_interfaces__msg__Imu__Sequence * lhs, const rov_debug_interfaces__msg__Imu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rov_debug_interfaces__msg__Imu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rov_debug_interfaces__msg__Imu__Sequence__copy(
  const rov_debug_interfaces__msg__Imu__Sequence * input,
  rov_debug_interfaces__msg__Imu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rov_debug_interfaces__msg__Imu);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rov_debug_interfaces__msg__Imu * data =
      (rov_debug_interfaces__msg__Imu *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rov_debug_interfaces__msg__Imu__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rov_debug_interfaces__msg__Imu__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rov_debug_interfaces__msg__Imu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
