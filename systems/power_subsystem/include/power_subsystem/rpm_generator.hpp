/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include "power_subsystem/generator.hpp"
#include "lotusim_common/logger.hpp"
#include <spdlog/spdlog.h>
#include <std_msgs/msg/float64.hpp>
#include <gz/common/Console.hh>
#include "lotusim_common/common.hpp"

namespace lotusim::gazebo
{

/**
 * @brief Generator with fuel consumption based on live RPM topic
 *
 * Subscribes to /<vessel_name>/rpm (std_msgs/Float64) and computes
 * power output using the propeller cubic law:
 *
 *   power_w = rated_output_w * (rpm / rated_rpm)³
 *
 * Fuel consumption per tick:
 *   fuel_consumed = (power_w * dt) / (efficiency * energy_density_j_per_l)
 *
 * When rpm = 0: no power produced, no fuel consumed
 *
 * SDF format (inside <lotusim_power> tag):
 *   <type>rpm_generator</type>
 *   <fuel_type>diesel</fuel_type>
 *   <fuel_capacity>500</fuel_capacity>
 *   <fuel_level_start>400</fuel_level_start>
 *   <rated_output_w>5000</rated_output_w>
 *   <rated_rpm>2000</rated_rpm>
 *   <efficiency>0.35</efficiency>
 *   <voltage_nominal>48.0</voltage_nominal>
 */
class RpmGenerator : public Generator
{
public:
    /**
     * @param name        generator name
     * @param _sdf        sdf
     * @param node        node from PowerManager
     * @param vessel_name for topic creation
     */
    RpmGenerator(
        std::string name,
        const sdf::ElementPtr& _sdf,
        rclcpp::Node::SharedPtr node,
        const std::string& vessel_name);

    ~RpmGenerator() override = default;

    // ----------------------------------------------------------------
    // Generator interface
    // ----------------------------------------------------------------

    /**
     * @brief Consume fuel based on current RPM over dt
     *       
     * @param currentA  total current drawn from generator (A)
     * @param dt        elapsed time since last tick (s)
     */
    void receiveLoad(float currentA, float dt) override;

    /**
     * @brief Returns power output based on current RPM
     *        Overrides Generator::availablePowerW() to use RPM-based model
     */
    float availablePowerW() const override;

private:
    void onRpm(const std_msgs::msg::Float64::SharedPtr msg);

    float m_rated_rpm{1000.0f};

    // latest RPM received from topic. Updated by onRpm callback
    std::atomic<float> m_current_rpm{0.0f};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_rpm_sub;

    std::shared_ptr<spdlog::logger> m_logger;
};

} // namespace lotusim::gazebo