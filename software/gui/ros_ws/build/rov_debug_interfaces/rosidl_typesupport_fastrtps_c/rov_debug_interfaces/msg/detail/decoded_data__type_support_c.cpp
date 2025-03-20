// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice
#include "rov_debug_interfaces/msg/detail/decoded_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rov_debug_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rov_debug_interfaces/msg/detail/decoded_data__struct.h"
#include "rov_debug_interfaces/msg/detail/decoded_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rov_debug_interfaces/msg/detail/imu__functions.h"  // imu

// forward declare type support functions
size_t get_serialized_size_rov_debug_interfaces__msg__Imu(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rov_debug_interfaces__msg__Imu(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rov_debug_interfaces, msg, Imu)();


using _DecodedData__ros_msg_type = rov_debug_interfaces__msg__DecodedData;

static bool _DecodedData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DecodedData__ros_msg_type * ros_message = static_cast<const _DecodedData__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: valid
  {
    cdr << (ros_message->valid ? true : false);
  }

  // Field name: imu
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rov_debug_interfaces, msg, Imu
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu, cdr))
    {
      return false;
    }
  }

  // Field name: thruster_current_1
  {
    cdr << ros_message->thruster_current_1;
  }

  // Field name: thruster_current_2
  {
    cdr << ros_message->thruster_current_2;
  }

  // Field name: thruster_current_3
  {
    cdr << ros_message->thruster_current_3;
  }

  // Field name: thruster_current_4
  {
    cdr << ros_message->thruster_current_4;
  }

  // Field name: thruster_current_5
  {
    cdr << ros_message->thruster_current_5;
  }

  // Field name: thruster_current_6
  {
    cdr << ros_message->thruster_current_6;
  }

  // Field name: thruster_pwm_1
  {
    cdr << ros_message->thruster_pwm_1;
  }

  // Field name: thruster_pwm_2
  {
    cdr << ros_message->thruster_pwm_2;
  }

  // Field name: thruster_pwm_3
  {
    cdr << ros_message->thruster_pwm_3;
  }

  // Field name: thruster_pwm_4
  {
    cdr << ros_message->thruster_pwm_4;
  }

  // Field name: thruster_pwm_5
  {
    cdr << ros_message->thruster_pwm_5;
  }

  // Field name: thruster_pwm_6
  {
    cdr << ros_message->thruster_pwm_6;
  }

  // Field name: indicator_1
  {
    cdr << ros_message->indicator_1;
  }

  // Field name: indicator_2
  {
    cdr << ros_message->indicator_2;
  }

  // Field name: indicator_3
  {
    cdr << ros_message->indicator_3;
  }

  // Field name: indicator_4
  {
    cdr << ros_message->indicator_4;
  }

  // Field name: indicator_5
  {
    cdr << ros_message->indicator_5;
  }

  // Field name: indicator_6
  {
    cdr << ros_message->indicator_6;
  }

  // Field name: heartbeat_1
  {
    cdr << (ros_message->heartbeat_1 ? true : false);
  }

  // Field name: heartbeat_2
  {
    cdr << (ros_message->heartbeat_2 ? true : false);
  }

  // Field name: heartbeat_3
  {
    cdr << (ros_message->heartbeat_3 ? true : false);
  }

  // Field name: heartbeat_4
  {
    cdr << (ros_message->heartbeat_4 ? true : false);
  }

  // Field name: connection_percentage_1
  {
    cdr << ros_message->connection_percentage_1;
  }

  // Field name: connection_percentage_2
  {
    cdr << ros_message->connection_percentage_2;
  }

  // Field name: connection_percentage_3
  {
    cdr << ros_message->connection_percentage_3;
  }

  // Field name: connection_percentage_4
  {
    cdr << ros_message->connection_percentage_4;
  }

  // Field name: arm_1
  {
    cdr << ros_message->arm_1;
  }

  // Field name: arm_2
  {
    cdr << ros_message->arm_2;
  }

  // Field name: arm_3
  {
    cdr << ros_message->arm_3;
  }

  // Field name: arm_4
  {
    cdr << ros_message->arm_4;
  }

  // Field name: rov_depth
  {
    cdr << ros_message->rov_depth;
  }

  return true;
}

static bool _DecodedData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DecodedData__ros_msg_type * ros_message = static_cast<_DecodedData__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid = tmp ? true : false;
  }

  // Field name: imu
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rov_debug_interfaces, msg, Imu
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu))
    {
      return false;
    }
  }

  // Field name: thruster_current_1
  {
    cdr >> ros_message->thruster_current_1;
  }

  // Field name: thruster_current_2
  {
    cdr >> ros_message->thruster_current_2;
  }

  // Field name: thruster_current_3
  {
    cdr >> ros_message->thruster_current_3;
  }

  // Field name: thruster_current_4
  {
    cdr >> ros_message->thruster_current_4;
  }

  // Field name: thruster_current_5
  {
    cdr >> ros_message->thruster_current_5;
  }

  // Field name: thruster_current_6
  {
    cdr >> ros_message->thruster_current_6;
  }

  // Field name: thruster_pwm_1
  {
    cdr >> ros_message->thruster_pwm_1;
  }

  // Field name: thruster_pwm_2
  {
    cdr >> ros_message->thruster_pwm_2;
  }

  // Field name: thruster_pwm_3
  {
    cdr >> ros_message->thruster_pwm_3;
  }

  // Field name: thruster_pwm_4
  {
    cdr >> ros_message->thruster_pwm_4;
  }

  // Field name: thruster_pwm_5
  {
    cdr >> ros_message->thruster_pwm_5;
  }

  // Field name: thruster_pwm_6
  {
    cdr >> ros_message->thruster_pwm_6;
  }

  // Field name: indicator_1
  {
    cdr >> ros_message->indicator_1;
  }

  // Field name: indicator_2
  {
    cdr >> ros_message->indicator_2;
  }

  // Field name: indicator_3
  {
    cdr >> ros_message->indicator_3;
  }

  // Field name: indicator_4
  {
    cdr >> ros_message->indicator_4;
  }

  // Field name: indicator_5
  {
    cdr >> ros_message->indicator_5;
  }

  // Field name: indicator_6
  {
    cdr >> ros_message->indicator_6;
  }

  // Field name: heartbeat_1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->heartbeat_1 = tmp ? true : false;
  }

  // Field name: heartbeat_2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->heartbeat_2 = tmp ? true : false;
  }

  // Field name: heartbeat_3
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->heartbeat_3 = tmp ? true : false;
  }

  // Field name: heartbeat_4
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->heartbeat_4 = tmp ? true : false;
  }

  // Field name: connection_percentage_1
  {
    cdr >> ros_message->connection_percentage_1;
  }

  // Field name: connection_percentage_2
  {
    cdr >> ros_message->connection_percentage_2;
  }

  // Field name: connection_percentage_3
  {
    cdr >> ros_message->connection_percentage_3;
  }

  // Field name: connection_percentage_4
  {
    cdr >> ros_message->connection_percentage_4;
  }

  // Field name: arm_1
  {
    cdr >> ros_message->arm_1;
  }

  // Field name: arm_2
  {
    cdr >> ros_message->arm_2;
  }

  // Field name: arm_3
  {
    cdr >> ros_message->arm_3;
  }

  // Field name: arm_4
  {
    cdr >> ros_message->arm_4;
  }

  // Field name: rov_depth
  {
    cdr >> ros_message->rov_depth;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rov_debug_interfaces
size_t get_serialized_size_rov_debug_interfaces__msg__DecodedData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DecodedData__ros_msg_type * ros_message = static_cast<const _DecodedData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valid
  {
    size_t item_size = sizeof(ros_message->valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu

  current_alignment += get_serialized_size_rov_debug_interfaces__msg__Imu(
    &(ros_message->imu), current_alignment);
  // field.name thruster_current_1
  {
    size_t item_size = sizeof(ros_message->thruster_current_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_current_2
  {
    size_t item_size = sizeof(ros_message->thruster_current_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_current_3
  {
    size_t item_size = sizeof(ros_message->thruster_current_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_current_4
  {
    size_t item_size = sizeof(ros_message->thruster_current_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_current_5
  {
    size_t item_size = sizeof(ros_message->thruster_current_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_current_6
  {
    size_t item_size = sizeof(ros_message->thruster_current_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_1
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_2
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_3
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_4
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_5
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name thruster_pwm_6
  {
    size_t item_size = sizeof(ros_message->thruster_pwm_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_1
  {
    size_t item_size = sizeof(ros_message->indicator_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_2
  {
    size_t item_size = sizeof(ros_message->indicator_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_3
  {
    size_t item_size = sizeof(ros_message->indicator_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_4
  {
    size_t item_size = sizeof(ros_message->indicator_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_5
  {
    size_t item_size = sizeof(ros_message->indicator_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_6
  {
    size_t item_size = sizeof(ros_message->indicator_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heartbeat_1
  {
    size_t item_size = sizeof(ros_message->heartbeat_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heartbeat_2
  {
    size_t item_size = sizeof(ros_message->heartbeat_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heartbeat_3
  {
    size_t item_size = sizeof(ros_message->heartbeat_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name heartbeat_4
  {
    size_t item_size = sizeof(ros_message->heartbeat_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name connection_percentage_1
  {
    size_t item_size = sizeof(ros_message->connection_percentage_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name connection_percentage_2
  {
    size_t item_size = sizeof(ros_message->connection_percentage_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name connection_percentage_3
  {
    size_t item_size = sizeof(ros_message->connection_percentage_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name connection_percentage_4
  {
    size_t item_size = sizeof(ros_message->connection_percentage_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm_1
  {
    size_t item_size = sizeof(ros_message->arm_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm_2
  {
    size_t item_size = sizeof(ros_message->arm_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm_3
  {
    size_t item_size = sizeof(ros_message->arm_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm_4
  {
    size_t item_size = sizeof(ros_message->arm_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rov_depth
  {
    size_t item_size = sizeof(ros_message->rov_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DecodedData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rov_debug_interfaces__msg__DecodedData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rov_debug_interfaces
size_t max_serialized_size_rov_debug_interfaces__msg__DecodedData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: imu
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_rov_debug_interfaces__msg__Imu(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: thruster_current_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_current_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_current_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_current_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_current_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_current_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: thruster_pwm_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: indicator_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: heartbeat_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: heartbeat_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: heartbeat_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: heartbeat_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: connection_percentage_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: connection_percentage_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: connection_percentage_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: connection_percentage_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: arm_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: arm_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: arm_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: arm_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rov_depth
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rov_debug_interfaces__msg__DecodedData;
    is_plain =
      (
      offsetof(DataType, rov_depth) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DecodedData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rov_debug_interfaces__msg__DecodedData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DecodedData = {
  "rov_debug_interfaces::msg",
  "DecodedData",
  _DecodedData__cdr_serialize,
  _DecodedData__cdr_deserialize,
  _DecodedData__get_serialized_size,
  _DecodedData__max_serialized_size
};

static rosidl_message_type_support_t _DecodedData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DecodedData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rov_debug_interfaces, msg, DecodedData)() {
  return &_DecodedData__type_support;
}

#if defined(__cplusplus)
}
#endif
