#ifndef __UNITY_INTERFACE_HPP_
#define __UNITY_INTERFACE_HPP_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <iostream>

#include <boost/asio.hpp>

class UnityInterface : public rclcpp::Node {
public:
    UnityInterface();

private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const;

private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_subscription;
};
#endif