// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rov_debug_interfaces/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rov_debug_interfaces
{

namespace msg
{

namespace builder
{

class Init_Imu_imu_yaw
{
public:
  explicit Init_Imu_imu_yaw(::rov_debug_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  ::rov_debug_interfaces::msg::Imu imu_yaw(::rov_debug_interfaces::msg::Imu::_imu_yaw_type arg)
  {
    msg_.imu_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

class Init_Imu_imu_pitch
{
public:
  explicit Init_Imu_imu_pitch(::rov_debug_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_imu_yaw imu_pitch(::rov_debug_interfaces::msg::Imu::_imu_pitch_type arg)
  {
    msg_.imu_pitch = std::move(arg);
    return Init_Imu_imu_yaw(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

class Init_Imu_imu_roll
{
public:
  explicit Init_Imu_imu_roll(::rov_debug_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_imu_pitch imu_roll(::rov_debug_interfaces::msg::Imu::_imu_roll_type arg)
  {
    msg_.imu_roll = std::move(arg);
    return Init_Imu_imu_pitch(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

class Init_Imu_acc_z
{
public:
  explicit Init_Imu_acc_z(::rov_debug_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_imu_roll acc_z(::rov_debug_interfaces::msg::Imu::_acc_z_type arg)
  {
    msg_.acc_z = std::move(arg);
    return Init_Imu_imu_roll(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

class Init_Imu_acc_y
{
public:
  explicit Init_Imu_acc_y(::rov_debug_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_acc_z acc_y(::rov_debug_interfaces::msg::Imu::_acc_y_type arg)
  {
    msg_.acc_y = std::move(arg);
    return Init_Imu_acc_z(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

class Init_Imu_acc_x
{
public:
  Init_Imu_acc_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Imu_acc_y acc_x(::rov_debug_interfaces::msg::Imu::_acc_x_type arg)
  {
    msg_.acc_x = std::move(arg);
    return Init_Imu_acc_y(msg_);
  }

private:
  ::rov_debug_interfaces::msg::Imu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rov_debug_interfaces::msg::Imu>()
{
  return rov_debug_interfaces::msg::builder::Init_Imu_acc_x();
}

}  // namespace rov_debug_interfaces

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_
