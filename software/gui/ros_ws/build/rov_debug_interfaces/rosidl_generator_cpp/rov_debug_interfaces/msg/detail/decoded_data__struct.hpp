// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rov_debug_interfaces:msg/DecodedData.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'imu'
#include "rov_debug_interfaces/msg/detail/imu__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rov_debug_interfaces__msg__DecodedData __attribute__((deprecated))
#else
# define DEPRECATED__rov_debug_interfaces__msg__DecodedData __declspec(deprecated)
#endif

namespace rov_debug_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DecodedData_
{
  using Type = DecodedData_<ContainerAllocator>;

  explicit DecodedData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->valid = false;
      this->thruster_current_1 = 0.0f;
      this->thruster_current_2 = 0.0f;
      this->thruster_current_3 = 0.0f;
      this->thruster_current_4 = 0.0f;
      this->thruster_current_5 = 0.0f;
      this->thruster_current_6 = 0.0f;
      this->thruster_pwm_1 = 0.0f;
      this->thruster_pwm_2 = 0.0f;
      this->thruster_pwm_3 = 0.0f;
      this->thruster_pwm_4 = 0.0f;
      this->thruster_pwm_5 = 0.0f;
      this->thruster_pwm_6 = 0.0f;
      this->indicator_1 = 0.0f;
      this->indicator_2 = 0.0f;
      this->indicator_3 = 0.0f;
      this->indicator_4 = 0.0f;
      this->indicator_5 = 0.0f;
      this->indicator_6 = 0.0f;
      this->heartbeat_1 = false;
      this->heartbeat_2 = false;
      this->heartbeat_3 = false;
      this->heartbeat_4 = false;
      this->connection_percentage_1 = 0.0f;
      this->connection_percentage_2 = 0.0f;
      this->connection_percentage_3 = 0.0f;
      this->connection_percentage_4 = 0.0f;
      this->arm_1 = 0.0f;
      this->arm_2 = 0.0f;
      this->arm_3 = 0.0f;
      this->arm_4 = 0.0f;
      this->rov_depth = 0.0f;
    }
  }

  explicit DecodedData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->valid = false;
      this->thruster_current_1 = 0.0f;
      this->thruster_current_2 = 0.0f;
      this->thruster_current_3 = 0.0f;
      this->thruster_current_4 = 0.0f;
      this->thruster_current_5 = 0.0f;
      this->thruster_current_6 = 0.0f;
      this->thruster_pwm_1 = 0.0f;
      this->thruster_pwm_2 = 0.0f;
      this->thruster_pwm_3 = 0.0f;
      this->thruster_pwm_4 = 0.0f;
      this->thruster_pwm_5 = 0.0f;
      this->thruster_pwm_6 = 0.0f;
      this->indicator_1 = 0.0f;
      this->indicator_2 = 0.0f;
      this->indicator_3 = 0.0f;
      this->indicator_4 = 0.0f;
      this->indicator_5 = 0.0f;
      this->indicator_6 = 0.0f;
      this->heartbeat_1 = false;
      this->heartbeat_2 = false;
      this->heartbeat_3 = false;
      this->heartbeat_4 = false;
      this->connection_percentage_1 = 0.0f;
      this->connection_percentage_2 = 0.0f;
      this->connection_percentage_3 = 0.0f;
      this->connection_percentage_4 = 0.0f;
      this->arm_1 = 0.0f;
      this->arm_2 = 0.0f;
      this->arm_3 = 0.0f;
      this->arm_4 = 0.0f;
      this->rov_depth = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _valid_type =
    bool;
  _valid_type valid;
  using _imu_type =
    rov_debug_interfaces::msg::Imu_<ContainerAllocator>;
  _imu_type imu;
  using _thruster_current_1_type =
    float;
  _thruster_current_1_type thruster_current_1;
  using _thruster_current_2_type =
    float;
  _thruster_current_2_type thruster_current_2;
  using _thruster_current_3_type =
    float;
  _thruster_current_3_type thruster_current_3;
  using _thruster_current_4_type =
    float;
  _thruster_current_4_type thruster_current_4;
  using _thruster_current_5_type =
    float;
  _thruster_current_5_type thruster_current_5;
  using _thruster_current_6_type =
    float;
  _thruster_current_6_type thruster_current_6;
  using _thruster_pwm_1_type =
    float;
  _thruster_pwm_1_type thruster_pwm_1;
  using _thruster_pwm_2_type =
    float;
  _thruster_pwm_2_type thruster_pwm_2;
  using _thruster_pwm_3_type =
    float;
  _thruster_pwm_3_type thruster_pwm_3;
  using _thruster_pwm_4_type =
    float;
  _thruster_pwm_4_type thruster_pwm_4;
  using _thruster_pwm_5_type =
    float;
  _thruster_pwm_5_type thruster_pwm_5;
  using _thruster_pwm_6_type =
    float;
  _thruster_pwm_6_type thruster_pwm_6;
  using _indicator_1_type =
    float;
  _indicator_1_type indicator_1;
  using _indicator_2_type =
    float;
  _indicator_2_type indicator_2;
  using _indicator_3_type =
    float;
  _indicator_3_type indicator_3;
  using _indicator_4_type =
    float;
  _indicator_4_type indicator_4;
  using _indicator_5_type =
    float;
  _indicator_5_type indicator_5;
  using _indicator_6_type =
    float;
  _indicator_6_type indicator_6;
  using _heartbeat_1_type =
    bool;
  _heartbeat_1_type heartbeat_1;
  using _heartbeat_2_type =
    bool;
  _heartbeat_2_type heartbeat_2;
  using _heartbeat_3_type =
    bool;
  _heartbeat_3_type heartbeat_3;
  using _heartbeat_4_type =
    bool;
  _heartbeat_4_type heartbeat_4;
  using _connection_percentage_1_type =
    float;
  _connection_percentage_1_type connection_percentage_1;
  using _connection_percentage_2_type =
    float;
  _connection_percentage_2_type connection_percentage_2;
  using _connection_percentage_3_type =
    float;
  _connection_percentage_3_type connection_percentage_3;
  using _connection_percentage_4_type =
    float;
  _connection_percentage_4_type connection_percentage_4;
  using _arm_1_type =
    float;
  _arm_1_type arm_1;
  using _arm_2_type =
    float;
  _arm_2_type arm_2;
  using _arm_3_type =
    float;
  _arm_3_type arm_3;
  using _arm_4_type =
    float;
  _arm_4_type arm_4;
  using _rov_depth_type =
    float;
  _rov_depth_type rov_depth;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__imu(
    const rov_debug_interfaces::msg::Imu_<ContainerAllocator> & _arg)
  {
    this->imu = _arg;
    return *this;
  }
  Type & set__thruster_current_1(
    const float & _arg)
  {
    this->thruster_current_1 = _arg;
    return *this;
  }
  Type & set__thruster_current_2(
    const float & _arg)
  {
    this->thruster_current_2 = _arg;
    return *this;
  }
  Type & set__thruster_current_3(
    const float & _arg)
  {
    this->thruster_current_3 = _arg;
    return *this;
  }
  Type & set__thruster_current_4(
    const float & _arg)
  {
    this->thruster_current_4 = _arg;
    return *this;
  }
  Type & set__thruster_current_5(
    const float & _arg)
  {
    this->thruster_current_5 = _arg;
    return *this;
  }
  Type & set__thruster_current_6(
    const float & _arg)
  {
    this->thruster_current_6 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_1(
    const float & _arg)
  {
    this->thruster_pwm_1 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_2(
    const float & _arg)
  {
    this->thruster_pwm_2 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_3(
    const float & _arg)
  {
    this->thruster_pwm_3 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_4(
    const float & _arg)
  {
    this->thruster_pwm_4 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_5(
    const float & _arg)
  {
    this->thruster_pwm_5 = _arg;
    return *this;
  }
  Type & set__thruster_pwm_6(
    const float & _arg)
  {
    this->thruster_pwm_6 = _arg;
    return *this;
  }
  Type & set__indicator_1(
    const float & _arg)
  {
    this->indicator_1 = _arg;
    return *this;
  }
  Type & set__indicator_2(
    const float & _arg)
  {
    this->indicator_2 = _arg;
    return *this;
  }
  Type & set__indicator_3(
    const float & _arg)
  {
    this->indicator_3 = _arg;
    return *this;
  }
  Type & set__indicator_4(
    const float & _arg)
  {
    this->indicator_4 = _arg;
    return *this;
  }
  Type & set__indicator_5(
    const float & _arg)
  {
    this->indicator_5 = _arg;
    return *this;
  }
  Type & set__indicator_6(
    const float & _arg)
  {
    this->indicator_6 = _arg;
    return *this;
  }
  Type & set__heartbeat_1(
    const bool & _arg)
  {
    this->heartbeat_1 = _arg;
    return *this;
  }
  Type & set__heartbeat_2(
    const bool & _arg)
  {
    this->heartbeat_2 = _arg;
    return *this;
  }
  Type & set__heartbeat_3(
    const bool & _arg)
  {
    this->heartbeat_3 = _arg;
    return *this;
  }
  Type & set__heartbeat_4(
    const bool & _arg)
  {
    this->heartbeat_4 = _arg;
    return *this;
  }
  Type & set__connection_percentage_1(
    const float & _arg)
  {
    this->connection_percentage_1 = _arg;
    return *this;
  }
  Type & set__connection_percentage_2(
    const float & _arg)
  {
    this->connection_percentage_2 = _arg;
    return *this;
  }
  Type & set__connection_percentage_3(
    const float & _arg)
  {
    this->connection_percentage_3 = _arg;
    return *this;
  }
  Type & set__connection_percentage_4(
    const float & _arg)
  {
    this->connection_percentage_4 = _arg;
    return *this;
  }
  Type & set__arm_1(
    const float & _arg)
  {
    this->arm_1 = _arg;
    return *this;
  }
  Type & set__arm_2(
    const float & _arg)
  {
    this->arm_2 = _arg;
    return *this;
  }
  Type & set__arm_3(
    const float & _arg)
  {
    this->arm_3 = _arg;
    return *this;
  }
  Type & set__arm_4(
    const float & _arg)
  {
    this->arm_4 = _arg;
    return *this;
  }
  Type & set__rov_depth(
    const float & _arg)
  {
    this->rov_depth = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> *;
  using ConstRawPtr =
    const rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rov_debug_interfaces__msg__DecodedData
    std::shared_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rov_debug_interfaces__msg__DecodedData
    std::shared_ptr<rov_debug_interfaces::msg::DecodedData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DecodedData_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->imu != other.imu) {
      return false;
    }
    if (this->thruster_current_1 != other.thruster_current_1) {
      return false;
    }
    if (this->thruster_current_2 != other.thruster_current_2) {
      return false;
    }
    if (this->thruster_current_3 != other.thruster_current_3) {
      return false;
    }
    if (this->thruster_current_4 != other.thruster_current_4) {
      return false;
    }
    if (this->thruster_current_5 != other.thruster_current_5) {
      return false;
    }
    if (this->thruster_current_6 != other.thruster_current_6) {
      return false;
    }
    if (this->thruster_pwm_1 != other.thruster_pwm_1) {
      return false;
    }
    if (this->thruster_pwm_2 != other.thruster_pwm_2) {
      return false;
    }
    if (this->thruster_pwm_3 != other.thruster_pwm_3) {
      return false;
    }
    if (this->thruster_pwm_4 != other.thruster_pwm_4) {
      return false;
    }
    if (this->thruster_pwm_5 != other.thruster_pwm_5) {
      return false;
    }
    if (this->thruster_pwm_6 != other.thruster_pwm_6) {
      return false;
    }
    if (this->indicator_1 != other.indicator_1) {
      return false;
    }
    if (this->indicator_2 != other.indicator_2) {
      return false;
    }
    if (this->indicator_3 != other.indicator_3) {
      return false;
    }
    if (this->indicator_4 != other.indicator_4) {
      return false;
    }
    if (this->indicator_5 != other.indicator_5) {
      return false;
    }
    if (this->indicator_6 != other.indicator_6) {
      return false;
    }
    if (this->heartbeat_1 != other.heartbeat_1) {
      return false;
    }
    if (this->heartbeat_2 != other.heartbeat_2) {
      return false;
    }
    if (this->heartbeat_3 != other.heartbeat_3) {
      return false;
    }
    if (this->heartbeat_4 != other.heartbeat_4) {
      return false;
    }
    if (this->connection_percentage_1 != other.connection_percentage_1) {
      return false;
    }
    if (this->connection_percentage_2 != other.connection_percentage_2) {
      return false;
    }
    if (this->connection_percentage_3 != other.connection_percentage_3) {
      return false;
    }
    if (this->connection_percentage_4 != other.connection_percentage_4) {
      return false;
    }
    if (this->arm_1 != other.arm_1) {
      return false;
    }
    if (this->arm_2 != other.arm_2) {
      return false;
    }
    if (this->arm_3 != other.arm_3) {
      return false;
    }
    if (this->arm_4 != other.arm_4) {
      return false;
    }
    if (this->rov_depth != other.rov_depth) {
      return false;
    }
    return true;
  }
  bool operator!=(const DecodedData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DecodedData_

// alias to use template instance with default allocator
using DecodedData =
  rov_debug_interfaces::msg::DecodedData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rov_debug_interfaces

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__DECODED_DATA__STRUCT_HPP_
