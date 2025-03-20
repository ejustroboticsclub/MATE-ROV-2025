// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Imu in the package rov_debug_interfaces.
typedef struct rov_debug_interfaces__msg__Imu
{
  float acc_x;
  float acc_y;
  float acc_z;
  float imu_roll;
  float imu_pitch;
  float imu_yaw;
} rov_debug_interfaces__msg__Imu;

// Struct for a sequence of rov_debug_interfaces__msg__Imu.
typedef struct rov_debug_interfaces__msg__Imu__Sequence
{
  rov_debug_interfaces__msg__Imu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rov_debug_interfaces__msg__Imu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
