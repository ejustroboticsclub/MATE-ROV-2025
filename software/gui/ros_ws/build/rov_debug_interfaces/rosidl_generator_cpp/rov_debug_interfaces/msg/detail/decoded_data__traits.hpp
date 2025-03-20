// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__TRAITS_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rov_debug_interfaces/msg/detail/decoded_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'imu'
#include "rov_debug_interfaces/msg/detail/imu__traits.hpp"

namespace rov_debug_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DecodedData & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: imu
  {
    out << "imu: ";
    to_flow_style_yaml(msg.imu, out);
    out << ", ";
  }

  // member: thruster_current_1
  {
    out << "thruster_current_1: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_1, out);
    out << ", ";
  }

  // member: thruster_current_2
  {
    out << "thruster_current_2: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_2, out);
    out << ", ";
  }

  // member: thruster_current_3
  {
    out << "thruster_current_3: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_3, out);
    out << ", ";
  }

  // member: thruster_current_4
  {
    out << "thruster_current_4: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_4, out);
    out << ", ";
  }

  // member: thruster_current_5
  {
    out << "thruster_current_5: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_5, out);
    out << ", ";
  }

  // member: thruster_current_6
  {
    out << "thruster_current_6: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_6, out);
    out << ", ";
  }

  // member: thruster_pwm_1
  {
    out << "thruster_pwm_1: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_1, out);
    out << ", ";
  }

  // member: thruster_pwm_2
  {
    out << "thruster_pwm_2: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_2, out);
    out << ", ";
  }

  // member: thruster_pwm_3
  {
    out << "thruster_pwm_3: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_3, out);
    out << ", ";
  }

  // member: thruster_pwm_4
  {
    out << "thruster_pwm_4: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_4, out);
    out << ", ";
  }

  // member: thruster_pwm_5
  {
    out << "thruster_pwm_5: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_5, out);
    out << ", ";
  }

  // member: thruster_pwm_6
  {
    out << "thruster_pwm_6: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_6, out);
    out << ", ";
  }

  // member: indicator_1
  {
    out << "indicator_1: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_1, out);
    out << ", ";
  }

  // member: indicator_2
  {
    out << "indicator_2: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_2, out);
    out << ", ";
  }

  // member: indicator_3
  {
    out << "indicator_3: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_3, out);
    out << ", ";
  }

  // member: indicator_4
  {
    out << "indicator_4: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_4, out);
    out << ", ";
  }

  // member: indicator_5
  {
    out << "indicator_5: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_5, out);
    out << ", ";
  }

  // member: indicator_6
  {
    out << "indicator_6: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_6, out);
    out << ", ";
  }

  // member: heartbeat_1
  {
    out << "heartbeat_1: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_1, out);
    out << ", ";
  }

  // member: heartbeat_2
  {
    out << "heartbeat_2: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_2, out);
    out << ", ";
  }

  // member: heartbeat_3
  {
    out << "heartbeat_3: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_3, out);
    out << ", ";
  }

  // member: heartbeat_4
  {
    out << "heartbeat_4: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_4, out);
    out << ", ";
  }

  // member: connection_percentage_1
  {
    out << "connection_percentage_1: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_1, out);
    out << ", ";
  }

  // member: connection_percentage_2
  {
    out << "connection_percentage_2: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_2, out);
    out << ", ";
  }

  // member: connection_percentage_3
  {
    out << "connection_percentage_3: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_3, out);
    out << ", ";
  }

  // member: connection_percentage_4
  {
    out << "connection_percentage_4: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_4, out);
    out << ", ";
  }

  // member: arm_1
  {
    out << "arm_1: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_1, out);
    out << ", ";
  }

  // member: arm_2
  {
    out << "arm_2: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_2, out);
    out << ", ";
  }

  // member: arm_3
  {
    out << "arm_3: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_3, out);
    out << ", ";
  }

  // member: arm_4
  {
    out << "arm_4: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_4, out);
    out << ", ";
  }

  // member: rov_depth
  {
    out << "rov_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.rov_depth, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DecodedData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu:\n";
    to_block_style_yaml(msg.imu, out, indentation + 2);
  }

  // member: thruster_current_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_1: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_1, out);
    out << "\n";
  }

  // member: thruster_current_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_2: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_2, out);
    out << "\n";
  }

  // member: thruster_current_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_3: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_3, out);
    out << "\n";
  }

  // member: thruster_current_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_4: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_4, out);
    out << "\n";
  }

  // member: thruster_current_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_5: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_5, out);
    out << "\n";
  }

  // member: thruster_current_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_current_6: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_current_6, out);
    out << "\n";
  }

  // member: thruster_pwm_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_1: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_1, out);
    out << "\n";
  }

  // member: thruster_pwm_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_2: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_2, out);
    out << "\n";
  }

  // member: thruster_pwm_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_3: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_3, out);
    out << "\n";
  }

  // member: thruster_pwm_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_4: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_4, out);
    out << "\n";
  }

  // member: thruster_pwm_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_5: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_5, out);
    out << "\n";
  }

  // member: thruster_pwm_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_pwm_6: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_pwm_6, out);
    out << "\n";
  }

  // member: indicator_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_1: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_1, out);
    out << "\n";
  }

  // member: indicator_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_2: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_2, out);
    out << "\n";
  }

  // member: indicator_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_3: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_3, out);
    out << "\n";
  }

  // member: indicator_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_4: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_4, out);
    out << "\n";
  }

  // member: indicator_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_5: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_5, out);
    out << "\n";
  }

  // member: indicator_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "indicator_6: ";
    rosidl_generator_traits::value_to_yaml(msg.indicator_6, out);
    out << "\n";
  }

  // member: heartbeat_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_1: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_1, out);
    out << "\n";
  }

  // member: heartbeat_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_2: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_2, out);
    out << "\n";
  }

  // member: heartbeat_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_3: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_3, out);
    out << "\n";
  }

  // member: heartbeat_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_4: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_4, out);
    out << "\n";
  }

  // member: connection_percentage_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_percentage_1: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_1, out);
    out << "\n";
  }

  // member: connection_percentage_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_percentage_2: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_2, out);
    out << "\n";
  }

  // member: connection_percentage_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_percentage_3: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_3, out);
    out << "\n";
  }

  // member: connection_percentage_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_percentage_4: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_percentage_4, out);
    out << "\n";
  }

  // member: arm_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_1: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_1, out);
    out << "\n";
  }

  // member: arm_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_2: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_2, out);
    out << "\n";
  }

  // member: arm_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_3: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_3, out);
    out << "\n";
  }

  // member: arm_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_4: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_4, out);
    out << "\n";
  }

  // member: rov_depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rov_depth: ";
    rosidl_generator_traits::value_to_yaml(msg.rov_depth, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DecodedData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rov_debug_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rov_debug_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rov_debug_interfaces::msg::DecodedData & msg,
  std::ostream & out, size_t indentation = 0)
{
  rov_debug_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rov_debug_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rov_debug_interfaces::msg::DecodedData & msg)
{
  return rov_debug_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rov_debug_interfaces::msg::DecodedData>()
{
  return "rov_debug_interfaces::msg::DecodedData";
}

template<>
inline const char * name<rov_debug_interfaces::msg::DecodedData>()
{
  return "rov_debug_interfaces/msg/DecodedData";
}

template<>
struct has_fixed_size<rov_debug_interfaces::msg::DecodedData>
  : std::integral_constant<bool, has_fixed_size<rov_debug_interfaces::msg::Imu>::value> {};

template<>
struct has_bounded_size<rov_debug_interfaces::msg::DecodedData>
  : std::integral_constant<bool, has_bounded_size<rov_debug_interfaces::msg::Imu>::value> {};

template<>
struct is_message<rov_debug_interfaces::msg::DecodedData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__TRAITS_HPP_
