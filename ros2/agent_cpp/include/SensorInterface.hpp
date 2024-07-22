#pragma once
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

class SensorInterface : public rclcpp::Node {
public:
    SensorInterface(std::string sensor_name, const rclcpp::NodeOptions &options)
        : rclcpp::Node(sensor_name, options) {};
    virtual void fetchData() = 0;
};