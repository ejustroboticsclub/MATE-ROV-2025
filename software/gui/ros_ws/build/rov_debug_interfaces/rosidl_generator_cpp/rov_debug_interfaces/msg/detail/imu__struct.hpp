// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rov_debug_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_HPP_
#define ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rov_debug_interfaces__msg__Imu __attribute__((deprecated))
#else
# define DEPRECATED__rov_debug_interfaces__msg__Imu __declspec(deprecated)
#endif

namespace rov_debug_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Imu_
{
  using Type = Imu_<ContainerAllocator>;

  explicit Imu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acc_x = 0.0f;
      this->acc_y = 0.0f;
      this->acc_z = 0.0f;
      this->imu_roll = 0.0f;
      this->imu_pitch = 0.0f;
      this->imu_yaw = 0.0f;
    }
  }

  explicit Imu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acc_x = 0.0f;
      this->acc_y = 0.0f;
      this->acc_z = 0.0f;
      this->imu_roll = 0.0f;
      this->imu_pitch = 0.0f;
      this->imu_yaw = 0.0f;
    }
  }

  // field types and members
  using _acc_x_type =
    float;
  _acc_x_type acc_x;
  using _acc_y_type =
    float;
  _acc_y_type acc_y;
  using _acc_z_type =
    float;
  _acc_z_type acc_z;
  using _imu_roll_type =
    float;
  _imu_roll_type imu_roll;
  using _imu_pitch_type =
    float;
  _imu_pitch_type imu_pitch;
  using _imu_yaw_type =
    float;
  _imu_yaw_type imu_yaw;

  // setters for named parameter idiom
  Type & set__acc_x(
    const float & _arg)
  {
    this->acc_x = _arg;
    return *this;
  }
  Type & set__acc_y(
    const float & _arg)
  {
    this->acc_y = _arg;
    return *this;
  }
  Type & set__acc_z(
    const float & _arg)
  {
    this->acc_z = _arg;
    return *this;
  }
  Type & set__imu_roll(
    const float & _arg)
  {
    this->imu_roll = _arg;
    return *this;
  }
  Type & set__imu_pitch(
    const float & _arg)
  {
    this->imu_pitch = _arg;
    return *this;
  }
  Type & set__imu_yaw(
    const float & _arg)
  {
    this->imu_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rov_debug_interfaces::msg::Imu_<ContainerAllocator> *;
  using ConstRawPtr =
    const rov_debug_interfaces::msg::Imu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rov_debug_interfaces::msg::Imu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rov_debug_interfaces::msg::Imu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rov_debug_interfaces__msg__Imu
    std::shared_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rov_debug_interfaces__msg__Imu
    std::shared_ptr<rov_debug_interfaces::msg::Imu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Imu_ & other) const
  {
    if (this->acc_x != other.acc_x) {
      return false;
    }
    if (this->acc_y != other.acc_y) {
      return false;
    }
    if (this->acc_z != other.acc_z) {
      return false;
    }
    if (this->imu_roll != other.imu_roll) {
      return false;
    }
    if (this->imu_pitch != other.imu_pitch) {
      return false;
    }
    if (this->imu_yaw != other.imu_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const Imu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Imu_

// alias to use template instance with default allocator
using Imu =
  rov_debug_interfaces::msg::Imu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rov_debug_interfaces

#endif  // ROV_DEBUG_INTERFACES__MSG__DETAIL__IMU__STRUCT_HPP_
