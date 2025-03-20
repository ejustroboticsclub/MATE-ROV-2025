// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__TRAITS_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rov_debug_interfaces/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rov_debug_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Imu & msg,
  std::ostream & out)
{
  out << "{";
  // member: acc_x
  {
    out << "acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_x, out);
    out << ", ";
  }

  // member: acc_y
  {
    out << "acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_y, out);
    out << ", ";
  }

  // member: acc_z
  {
    out << "acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_z, out);
    out << ", ";
  }

  // member: imu_roll
  {
    out << "imu_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_roll, out);
    out << ", ";
  }

  // member: imu_pitch
  {
    out << "imu_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_pitch, out);
    out << ", ";
  }

  // member: imu_yaw
  {
    out << "imu_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Imu & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: acc_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_x, out);
    out << "\n";
  }

  // member: acc_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_y, out);
    out << "\n";
  }

  // member: acc_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.acc_z, out);
    out << "\n";
  }

  // member: imu_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_roll, out);
    out << "\n";
  }

  // member: imu_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_pitch, out);
    out << "\n";
  }

  // member: imu_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Imu & msg, bool use_flow_style = false)
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
  const rov_debug_interfaces::msg::Imu & msg,
  std::ostream & out, size_t indentation = 0)
{
  rov_debug_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rov_debug_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rov_debug_interfaces::msg::Imu & msg)
{
  return rov_debug_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rov_debug_interfaces::msg::Imu>()
{
  return "rov_debug_interfaces::msg::Imu";
}

template<>
inline const char * name<rov_debug_interfaces::msg::Imu>()
{
  return "rov_debug_interfaces/msg/Imu";
}

template<>
struct has_fixed_size<rov_debug_interfaces::msg::Imu>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rov_debug_interfaces::msg::Imu>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rov_debug_interfaces::msg::Imu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__TRAITS_HPP_
