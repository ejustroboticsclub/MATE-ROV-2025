// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__BUILDER_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rov_debug_interfaces/msg/detail/decoded_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rov_debug_interfaces
{

namespace msg
{

namespace builder
{

class Init_DecodedData_rov_depth
{
public:
  explicit Init_DecodedData_rov_depth(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  ::rov_debug_interfaces::msg::DecodedData rov_depth(::rov_debug_interfaces::msg::DecodedData::_rov_depth_type arg)
  {
    msg_.rov_depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_arm_4
{
public:
  explicit Init_DecodedData_arm_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_rov_depth arm_4(::rov_debug_interfaces::msg::DecodedData::_arm_4_type arg)
  {
    msg_.arm_4 = std::move(arg);
    return Init_DecodedData_rov_depth(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_arm_3
{
public:
  explicit Init_DecodedData_arm_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_arm_4 arm_3(::rov_debug_interfaces::msg::DecodedData::_arm_3_type arg)
  {
    msg_.arm_3 = std::move(arg);
    return Init_DecodedData_arm_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_arm_2
{
public:
  explicit Init_DecodedData_arm_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_arm_3 arm_2(::rov_debug_interfaces::msg::DecodedData::_arm_2_type arg)
  {
    msg_.arm_2 = std::move(arg);
    return Init_DecodedData_arm_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_arm_1
{
public:
  explicit Init_DecodedData_arm_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_arm_2 arm_1(::rov_debug_interfaces::msg::DecodedData::_arm_1_type arg)
  {
    msg_.arm_1 = std::move(arg);
    return Init_DecodedData_arm_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_connection_percentage_4
{
public:
  explicit Init_DecodedData_connection_percentage_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_arm_1 connection_percentage_4(::rov_debug_interfaces::msg::DecodedData::_connection_percentage_4_type arg)
  {
    msg_.connection_percentage_4 = std::move(arg);
    return Init_DecodedData_arm_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_connection_percentage_3
{
public:
  explicit Init_DecodedData_connection_percentage_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_connection_percentage_4 connection_percentage_3(::rov_debug_interfaces::msg::DecodedData::_connection_percentage_3_type arg)
  {
    msg_.connection_percentage_3 = std::move(arg);
    return Init_DecodedData_connection_percentage_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_connection_percentage_2
{
public:
  explicit Init_DecodedData_connection_percentage_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_connection_percentage_3 connection_percentage_2(::rov_debug_interfaces::msg::DecodedData::_connection_percentage_2_type arg)
  {
    msg_.connection_percentage_2 = std::move(arg);
    return Init_DecodedData_connection_percentage_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_connection_percentage_1
{
public:
  explicit Init_DecodedData_connection_percentage_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_connection_percentage_2 connection_percentage_1(::rov_debug_interfaces::msg::DecodedData::_connection_percentage_1_type arg)
  {
    msg_.connection_percentage_1 = std::move(arg);
    return Init_DecodedData_connection_percentage_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_heartbeat_4
{
public:
  explicit Init_DecodedData_heartbeat_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_connection_percentage_1 heartbeat_4(::rov_debug_interfaces::msg::DecodedData::_heartbeat_4_type arg)
  {
    msg_.heartbeat_4 = std::move(arg);
    return Init_DecodedData_connection_percentage_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_heartbeat_3
{
public:
  explicit Init_DecodedData_heartbeat_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_heartbeat_4 heartbeat_3(::rov_debug_interfaces::msg::DecodedData::_heartbeat_3_type arg)
  {
    msg_.heartbeat_3 = std::move(arg);
    return Init_DecodedData_heartbeat_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_heartbeat_2
{
public:
  explicit Init_DecodedData_heartbeat_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_heartbeat_3 heartbeat_2(::rov_debug_interfaces::msg::DecodedData::_heartbeat_2_type arg)
  {
    msg_.heartbeat_2 = std::move(arg);
    return Init_DecodedData_heartbeat_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_heartbeat_1
{
public:
  explicit Init_DecodedData_heartbeat_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_heartbeat_2 heartbeat_1(::rov_debug_interfaces::msg::DecodedData::_heartbeat_1_type arg)
  {
    msg_.heartbeat_1 = std::move(arg);
    return Init_DecodedData_heartbeat_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_6
{
public:
  explicit Init_DecodedData_indicator_6(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_heartbeat_1 indicator_6(::rov_debug_interfaces::msg::DecodedData::_indicator_6_type arg)
  {
    msg_.indicator_6 = std::move(arg);
    return Init_DecodedData_heartbeat_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_5
{
public:
  explicit Init_DecodedData_indicator_5(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_6 indicator_5(::rov_debug_interfaces::msg::DecodedData::_indicator_5_type arg)
  {
    msg_.indicator_5 = std::move(arg);
    return Init_DecodedData_indicator_6(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_4
{
public:
  explicit Init_DecodedData_indicator_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_5 indicator_4(::rov_debug_interfaces::msg::DecodedData::_indicator_4_type arg)
  {
    msg_.indicator_4 = std::move(arg);
    return Init_DecodedData_indicator_5(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_3
{
public:
  explicit Init_DecodedData_indicator_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_4 indicator_3(::rov_debug_interfaces::msg::DecodedData::_indicator_3_type arg)
  {
    msg_.indicator_3 = std::move(arg);
    return Init_DecodedData_indicator_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_2
{
public:
  explicit Init_DecodedData_indicator_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_3 indicator_2(::rov_debug_interfaces::msg::DecodedData::_indicator_2_type arg)
  {
    msg_.indicator_2 = std::move(arg);
    return Init_DecodedData_indicator_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_indicator_1
{
public:
  explicit Init_DecodedData_indicator_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_2 indicator_1(::rov_debug_interfaces::msg::DecodedData::_indicator_1_type arg)
  {
    msg_.indicator_1 = std::move(arg);
    return Init_DecodedData_indicator_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_6
{
public:
  explicit Init_DecodedData_thruster_pwm_6(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_indicator_1 thruster_pwm_6(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_6_type arg)
  {
    msg_.thruster_pwm_6 = std::move(arg);
    return Init_DecodedData_indicator_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_5
{
public:
  explicit Init_DecodedData_thruster_pwm_5(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_6 thruster_pwm_5(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_5_type arg)
  {
    msg_.thruster_pwm_5 = std::move(arg);
    return Init_DecodedData_thruster_pwm_6(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_4
{
public:
  explicit Init_DecodedData_thruster_pwm_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_5 thruster_pwm_4(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_4_type arg)
  {
    msg_.thruster_pwm_4 = std::move(arg);
    return Init_DecodedData_thruster_pwm_5(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_3
{
public:
  explicit Init_DecodedData_thruster_pwm_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_4 thruster_pwm_3(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_3_type arg)
  {
    msg_.thruster_pwm_3 = std::move(arg);
    return Init_DecodedData_thruster_pwm_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_2
{
public:
  explicit Init_DecodedData_thruster_pwm_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_3 thruster_pwm_2(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_2_type arg)
  {
    msg_.thruster_pwm_2 = std::move(arg);
    return Init_DecodedData_thruster_pwm_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_pwm_1
{
public:
  explicit Init_DecodedData_thruster_pwm_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_2 thruster_pwm_1(::rov_debug_interfaces::msg::DecodedData::_thruster_pwm_1_type arg)
  {
    msg_.thruster_pwm_1 = std::move(arg);
    return Init_DecodedData_thruster_pwm_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_6
{
public:
  explicit Init_DecodedData_thruster_current_6(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_pwm_1 thruster_current_6(::rov_debug_interfaces::msg::DecodedData::_thruster_current_6_type arg)
  {
    msg_.thruster_current_6 = std::move(arg);
    return Init_DecodedData_thruster_pwm_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_5
{
public:
  explicit Init_DecodedData_thruster_current_5(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_6 thruster_current_5(::rov_debug_interfaces::msg::DecodedData::_thruster_current_5_type arg)
  {
    msg_.thruster_current_5 = std::move(arg);
    return Init_DecodedData_thruster_current_6(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_4
{
public:
  explicit Init_DecodedData_thruster_current_4(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_5 thruster_current_4(::rov_debug_interfaces::msg::DecodedData::_thruster_current_4_type arg)
  {
    msg_.thruster_current_4 = std::move(arg);
    return Init_DecodedData_thruster_current_5(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_3
{
public:
  explicit Init_DecodedData_thruster_current_3(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_4 thruster_current_3(::rov_debug_interfaces::msg::DecodedData::_thruster_current_3_type arg)
  {
    msg_.thruster_current_3 = std::move(arg);
    return Init_DecodedData_thruster_current_4(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_2
{
public:
  explicit Init_DecodedData_thruster_current_2(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_3 thruster_current_2(::rov_debug_interfaces::msg::DecodedData::_thruster_current_2_type arg)
  {
    msg_.thruster_current_2 = std::move(arg);
    return Init_DecodedData_thruster_current_3(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_thruster_current_1
{
public:
  explicit Init_DecodedData_thruster_current_1(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_2 thruster_current_1(::rov_debug_interfaces::msg::DecodedData::_thruster_current_1_type arg)
  {
    msg_.thruster_current_1 = std::move(arg);
    return Init_DecodedData_thruster_current_2(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_imu
{
public:
  explicit Init_DecodedData_imu(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_thruster_current_1 imu(::rov_debug_interfaces::msg::DecodedData::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_DecodedData_thruster_current_1(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_valid
{
public:
  explicit Init_DecodedData_valid(::rov_debug_interfaces::msg::DecodedData & msg)
  : msg_(msg)
  {}
  Init_DecodedData_imu valid(::rov_debug_interfaces::msg::DecodedData::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_DecodedData_imu(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

class Init_DecodedData_id
{
public:
  Init_DecodedData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DecodedData_valid id(::rov_debug_interfaces::msg::DecodedData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DecodedData_valid(msg_);
  }

private:
  ::rov_debug_interfaces::msg::DecodedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rov_debug_interfaces::msg::DecodedData>()
{
  return rov_debug_interfaces::msg::builder::Init_DecodedData_id();
}

}  // namespace rov_debug_interfaces

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__BUILDER_HPP_
