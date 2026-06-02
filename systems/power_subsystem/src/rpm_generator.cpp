/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/rpm_generator.hpp"
#include "power_subsystem/fuel_properties.hpp"
#include <algorithm>
#include <cmath>
#include "lotusim_common/common.hpp"

// RpmGenerator: fuel consumption is driven by rpm (vessel_name/voltage)

namespace lotusim::gazebo
{

RpmGenerator::RpmGenerator(
    std::string name,
    const sdf::ElementPtr& _sdf,
    rclcpp::Node::SharedPtr node,
    const std::string& vessel_name)
    : Generator(
        std::move(name),
        std::move(node),
        _sdf->Get<float>("fuel_level_start",   400.0f).first,
        _sdf->Get<float>("fuel_capacity",      500.0f).first,
        _sdf->Get<float>("rated_output_w",    5000.0f).first,
        _sdf->Get<float>("efficiency",           0.35f).first,
        _sdf->Get<float>("voltage_nominal",     48.0f).first,
        _sdf->Get<std::string>("fuel_type", "diesel").first)
    , m_rated_rpm(_sdf->Get<float>("rated_rpm", 1000.0f).first)
{
    const std::string loggerName = "rpmGenerator_" + Generator::name();
    m_logger = logger::createConsoleAndFileLogger(
        loggerName, loggerName + ".txt");

    // subscribe to /<vessel_name>/rpm
    const std::string rpm_topic = "/" + vessel_name + "/rpm";
    m_rpm_sub = PowerProvider::m_node->create_subscription<std_msgs::msg::Float64>(
        rpm_topic,
        10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            onRpm(msg);
        });

    m_logger->info(
        "RpmGenerator [{}]: initialised : "
        "fuel {:.1f}/{:.1f} L type={} rated {:.0f} W "
        "rated_rpm={:.0f} efficiency={:.2f} voltage={:.1f} V "
        "subscribed to [{}]",
        Generator::name(),
        m_fuel_level, m_fuel_capacity, m_fuel_type,
        m_rated_output_W, m_rated_rpm, m_efficiency,
        m_voltage_nominal, rpm_topic);
}

void RpmGenerator::onRpm(const std_msgs::msg::Float64::SharedPtr msg)
{
    m_current_rpm.store(static_cast<float>(msg->data));
    m_logger->debug(
        "RpmGenerator [{}]: received rpm={:.1f}",
        Generator::name(), msg->data);
}

float RpmGenerator::availablePowerW() const
{
    if (isDepleted()) { return 0.0f; }

    const float rpm = m_current_rpm.load();
    if (rpm <= 0.0f) { return 0.0f; }

    // cubic law: power scales with rpm³
    const float ratio = rpm / m_rated_rpm;
    return m_rated_output_W * ratio * ratio * ratio;
}

void RpmGenerator::receiveLoad(float /*currentA*/, float dt)
{
    const float rpm = m_current_rpm.load();

    // engine off -> no fuel consumed, no power produced
    if (rpm <= 0.0f || isDepleted()) {
        return;
    }

    // power produced
    const float ratio   = rpm / m_rated_rpm;
    const float power_w = m_rated_output_W * ratio * ratio * ratio;

    // fuel consumption
    const float energyDensity = getFuelEnergyDensity(m_fuel_type);
    const float fuelConsumed  = (power_w * dt) / (m_efficiency * energyDensity);

    m_fuel_level = std::max(0.0f, m_fuel_level - fuelConsumed);

    m_logger->debug(
        "RpmGenerator [{}]: rpm={:.1f} power={:.1f} W "
        "fuel_consumed={:.6f} L remaining={:.3f}/{:.3f} L ({:.1f}%)",
        Generator::name(),
        rpm, power_w, fuelConsumed,
        m_fuel_level, m_fuel_capacity, fuelRatio() * 100.0f);

    if (isDepleted()) {
        m_logger->warn(
            "RpmGenerator [{}]: fuel depleted",
            Generator::name());
    }
}

} // namespace lotusim::gazebo