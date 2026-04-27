/*
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "power_subsystem/sensor_power_component.hpp"

#include "rclcpp/logging.hpp"

namespace lotusim::power_subsystem
{

SensorPowerComponent::SensorPowerComponent(
    rclcpp::Node::SharedPtr node,
    const std::string& sensor_name,
    const std::string& current_topic,
    const std::string& voltage_topic,
    float rated_power_w)
: PowerComponent(std::move(node), sensor_name, current_topic, voltage_topic),
  rated_power_w_(rated_power_w)
{
    RCLCPP_INFO(
        node_->get_logger(),
        "[SensorPowerComponent] '%s' initialised with rated power: %.2f W",
        component_name().c_str(),
        rated_power_w_);
}

float SensorPowerComponent::power_consume() const
{
    if (!is_active()) {
        return 0.0f;
    }

    return rated_power_w_;
}

} // namespace lotusim::power_subsystem