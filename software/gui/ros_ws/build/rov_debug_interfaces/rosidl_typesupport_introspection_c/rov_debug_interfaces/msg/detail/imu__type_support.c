// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rov_debug_interfaces/msg/detail/imu__rosidl_typesupport_introspection_c.h"
#include "rov_debug_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rov_debug_interfaces/msg/detail/imu__functions.h"
#include "rov_debug_interfaces/msg/detail/imu__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rov_debug_interfaces__msg__Imu__init(message_memory);
}

void rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_fini_function(void * message_memory)
{
  rov_debug_interfaces__msg__Imu__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_member_array[6] = {
  {
    "acc_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, acc_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, acc_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, acc_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, imu_roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, imu_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rov_debug_interfaces__msg__Imu, imu_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_members = {
  "rov_debug_interfaces__msg",  // message namespace
  "Imu",  // message name
  6,  // number of fields
  sizeof(rov_debug_interfaces__msg__Imu),
  rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_member_array,  // message members
  rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_init_function,  // function to initialize message memory (memory has to be allocated)
  rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_type_support_handle = {
  0,
  &rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rov_debug_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rov_debug_interfaces, msg, Imu)() {
  if (!rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_type_support_handle.typesupport_identifier) {
    rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rov_debug_interfaces__msg__Imu__rosidl_typesupport_introspection_c__Imu_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
