/*
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include "power_subsystem/power_component.hpp"

namespace lotusim::power_subsystem
{

/**
 * @brief Power component representing a sensor on the vessel
 * A sensor draws a fixed rated power when active
 */
class SensorPowerComponent final : public PowerComponent
{
public:
    /**
     * @param node              shared ROS 2 node for this vessel
     * @param sensor_name       identifier e.g. "lidar_front"
     * @param current_topic     FMU topic for current e.g. "/hectate/OUT_I"
     * @param voltage_topic     FMU topic for voltage e.g. "/hectate/OUT_U"
     * @param rated_power_w     rated power consumption in Watts e.g. 25.0f for a lidar
     */
    explicit SensorPowerComponent(
        rclcpp::Node::SharedPtr node,
        const std::string& sensor_name,
        const std::string& current_topic,
        const std::string& voltage_topic,
        float rated_power_w
    );

    ~SensorPowerComponent() override = default;

    /**
     * @brief Returns the rated power draw [W] when active, 0.0 when inactive.
     */
    float power_consume() const override;

private:
    float rated_power_w_;
};

} // namespace lotusim::power_subsystem