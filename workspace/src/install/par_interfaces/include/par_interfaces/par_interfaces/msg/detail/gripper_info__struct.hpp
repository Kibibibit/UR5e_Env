// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from par_interfaces:msg/GripperInfo.idl
// generated code does not contain a copyright notice

#ifndef PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__STRUCT_HPP_
#define PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__par_interfaces__msg__GripperInfo __attribute__((deprecated))
#else
# define DEPRECATED__par_interfaces__msg__GripperInfo __declspec(deprecated)
#endif

namespace par_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperInfo_
{
  using Type = GripperInfo_<ContainerAllocator>;

  explicit GripperInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gripper_type = "";
      this->ip = "";
      this->port = "";
      this->max_width = 0l;
      this->max_force = 0l;
    }
  }

  explicit GripperInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : gripper_type(_alloc),
    ip(_alloc),
    port(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gripper_type = "";
      this->ip = "";
      this->port = "";
      this->max_width = 0l;
      this->max_force = 0l;
    }
  }

  // field types and members
  using _gripper_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gripper_type_type gripper_type;
  using _ip_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ip_type ip;
  using _port_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _port_type port;
  using _max_width_type =
    int32_t;
  _max_width_type max_width;
  using _max_force_type =
    int32_t;
  _max_force_type max_force;

  // setters for named parameter idiom
  Type & set__gripper_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gripper_type = _arg;
    return *this;
  }
  Type & set__ip(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ip = _arg;
    return *this;
  }
  Type & set__port(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->port = _arg;
    return *this;
  }
  Type & set__max_width(
    const int32_t & _arg)
  {
    this->max_width = _arg;
    return *this;
  }
  Type & set__max_force(
    const int32_t & _arg)
  {
    this->max_force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    par_interfaces::msg::GripperInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const par_interfaces::msg::GripperInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      par_interfaces::msg::GripperInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      par_interfaces::msg::GripperInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__par_interfaces__msg__GripperInfo
    std::shared_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__par_interfaces__msg__GripperInfo
    std::shared_ptr<par_interfaces::msg::GripperInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperInfo_ & other) const
  {
    if (this->gripper_type != other.gripper_type) {
      return false;
    }
    if (this->ip != other.ip) {
      return false;
    }
    if (this->port != other.port) {
      return false;
    }
    if (this->max_width != other.max_width) {
      return false;
    }
    if (this->max_force != other.max_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperInfo_

// alias to use template instance with default allocator
using GripperInfo =
  par_interfaces::msg::GripperInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace par_interfaces

#endif  // PAR_INTERFACES__MSG__DETAIL__GRIPPER_INFO__STRUCT_HPP_
