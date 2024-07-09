
#ifndef ROS_GZ_BRIDGE__CONVERT__LIQUIDAI_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__LIQUIDAI_MSGS_HPP_

// Gazebo Msgs
#include <gz_liquidai_msgs/msgs/aismsg.pb.h>
#include <gz_liquidai_msgs/msgs/xdyncmdmsg.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose.pb.h>

// ROS 2 messages
#include <liquidai_msgs/msg/ais.hpp>
#include <liquidai_msgs/msg/ais_array.hpp>
#include <liquidai_msgs/msg/float_stamped.hpp>
#include <liquidai_msgs/msg/gps.hpp>
#include <liquidai_msgs/msg/xdyn_thrustercmd.hpp>
#include <liquidai_msgs/msg/xdyncmd.hpp>
#include <liquidai_msgs/msg/entity_position.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge {
// nav_msgs
template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::GPS &ros_msg, gz::msgs::NavSat &gz_msg);

template <>
void convert_gz_to_ros(
    const gz::msgs::NavSat &gz_msg, liquidai_msgs::msg::GPS &ros_msg);

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::AISArray &ros_msg,
    gz_liquidai_msgs::msgs::AISArray &gz_msg);

template <>
void convert_gz_to_ros(
    const gz_liquidai_msgs::msgs::AISArray &gz_msg,
    liquidai_msgs::msg::AISArray &ros_msg);

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::FloatStamped &ros_msg, gz::msgs::Double &gz_msg);

template <>
void convert_gz_to_ros(
    const gz::msgs::Double &gz_msg, liquidai_msgs::msg::FloatStamped &ros_msg);

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::Xdyncmd &ros_msg,
    gz_liquidai_msgs::msgs::XdynCmd &gz_msg);

template <>
void convert_gz_to_ros(
    const gz_liquidai_msgs::msgs::XdynCmd &gz_msg,
    liquidai_msgs::msg::Xdyncmd &ros_msg);

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::EntityPosition &ros_msg, gz::msgs::Pose &gz_msg);

template <>
void convert_gz_to_ros(
    const gz::msgs::Pose &gz_msg, liquidai_msgs::msg::EntityPosition &ros_msg);

} // namespace ros_gz_bridge

#endif // ROS_GZ_BRIDGE__CONVERT__LIQUIDAI_MSGS_HPP_
