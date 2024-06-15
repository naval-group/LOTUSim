
#ifndef ROS_GZ_BRIDGE__CONVERT__UUV_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__UUV_MSGS_HPP_

// Gazebo Msgs
#include <gz/msgs/double.pb.h>

// ROS 2 messages
#include <liquidai_msgs/msg/float_stamped.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge {
// nav_msgs
template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::FloatStamped &ros_msg, gz::msgs::Double &gz_msg);

template <>
void convert_gz_to_ros(
    const gz::msgs::Double &gz_msg, liquidai_msgs::msg::FloatStamped &ros_msg);

} // namespace ros_gz_bridge

#endif // ROS_GZ_BRIDGE__CONVERT__UUV_MSGS_HPP_
