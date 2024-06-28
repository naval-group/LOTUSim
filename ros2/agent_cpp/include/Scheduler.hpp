#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <chrono>
#include <composition_interfaces/srv/load_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

class Scheduler : public rclcpp::Node {
public:
    Scheduler();
};