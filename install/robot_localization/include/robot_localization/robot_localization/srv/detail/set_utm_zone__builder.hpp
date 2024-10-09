// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_localization:srv/SetUTMZone.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__BUILDER_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_localization/srv/detail/set_utm_zone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_localization
{

namespace srv
{

namespace builder
{

class Init_SetUTMZone_Request_utm_zone
{
public:
  Init_SetUTMZone_Request_utm_zone()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_localization::srv::SetUTMZone_Request utm_zone(::robot_localization::srv::SetUTMZone_Request::_utm_zone_type arg)
  {
    msg_.utm_zone = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_localization::srv::SetUTMZone_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetUTMZone_Request>()
{
  return robot_localization::srv::builder::Init_SetUTMZone_Request_utm_zone();
}

}  // namespace robot_localization


namespace robot_localization
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_localization::srv::SetUTMZone_Response>()
{
  return ::robot_localization::srv::SetUTMZone_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__BUILDER_HPP_
