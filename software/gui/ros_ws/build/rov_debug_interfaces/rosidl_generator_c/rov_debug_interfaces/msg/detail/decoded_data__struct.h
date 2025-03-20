// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_H_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imu'
#include "rov_debug_interfaces/msg/detail/imu__struct.h"

/// Struct defined in msg/DecodedData in the package rov_debug_interfaces.
typedef struct rov_debug_interfaces__msg__DecodedData
{
  int32_t id;
  bool valid;
  rov_debug_interfaces__msg__Imu imu;
  float thruster_current_1;
  float thruster_current_2;
  float thruster_current_3;
  float thruster_current_4;
  float thruster_current_5;
  float thruster_current_6;
  float thruster_pwm_1;
  float thruster_pwm_2;
  float thruster_pwm_3;
  float thruster_pwm_4;
  float thruster_pwm_5;
  float thruster_pwm_6;
  float indicator_1;
  float indicator_2;
  float indicator_3;
  float indicator_4;
  float indicator_5;
  float indicator_6;
  bool heartbeat_1;
  bool heartbeat_2;
  bool heartbeat_3;
  bool heartbeat_4;
  float connection_percentage_1;
  float connection_percentage_2;
  float connection_percentage_3;
  float connection_percentage_4;
  float arm_1;
  float arm_2;
  float arm_3;
  float arm_4;
  float rov_depth;
} rov_debug_interfaces__msg__DecodedData;

// Struct for a sequence of rov_debug_interfaces__msg__DecodedData.
typedef struct rov_debug_interfaces__msg__DecodedData__Sequence
{
  rov_debug_interfaces__msg__DecodedData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rov_debug_interfaces__msg__DecodedData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_H_
