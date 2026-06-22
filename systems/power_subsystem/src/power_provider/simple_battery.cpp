/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_provider/simple_battery.hpp"

#include <algorithm>

#include "lotusim_common/common.hpp"

namespace lotusim::gazebo {

SimpleBattery::SimpleBattery(
    const std::string& battery_name,
    const std::string& vessel_name,
    const sdf::ElementPtr& _sdf,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<spdlog::logger> logger)
    : Battery(
          std::move(battery_name),
          std::move(vessel_name),
          _sdf,
          std::move(node),
          logger)
    , m_voltage(0.0f)
    , m_voltage_nominal(_sdf->Get<float>("voltage_nominal", 48.0f).first)
    , m_remainingAh(
          _sdf->Get<float>("capacity_ah", 100.0f).first *
          _sdf->Get<float>("initial_soc", 1.0f).first)
{
    m_voltage =
        m_voltageMin + m_initialSoc * (m_voltage_nominal - m_voltageMin);
    m_provider_type = ProviderType::SimpleBattery;

    m_logger->info(
        "SimpleBattery [{},{}]: initialised with voltage {}, initial soc {}.",
        m_provider_name,
        m_vessel_name,
        m_voltage,
        getStateOfCharge());
}

void SimpleBattery::receiveLoad(float currentA, float dt)
{
    if (currentA > 0) {
        m_remainingAh -= currentA * dt;
        m_remainingAh = std::max(m_remainingAh, 0.0f);
    } else {
        m_remainingAh += currentA * dt;
        m_remainingAh = std::min(m_remainingAh, m_capacityAh);
    }
    updateVoltage();
    m_logger->debug(
        "SimpleBattery [{},{}]: received load {:.3f} A dt={:.4f} s "
        "remaining={:.3f} Ah SOC={:.3f} voltage={:.2f} V",
        m_provider_name,
        m_vessel_name,
        currentA,
        dt,
        m_remainingAh,
        getStateOfCharge(),
        voltage());
}

void SimpleBattery::updateVoltage()
{
    // voltage scales linearly with SOC between voltage_min and voltage_nominal
    // this is a simplification
    m_voltage =
        m_voltageMin + getStateOfCharge() * (m_voltage_nominal - m_voltageMin);
}

float SimpleBattery::voltage() const
{
    return m_voltage;
}

float SimpleBattery::getStateOfCharge() const
{
    if (m_capacityAh <= 0.0f) {
        return 0.0f;
    }
    return std::clamp(m_remainingAh / m_capacityAh, 0.0f, 1.0f);
}

// for now - energy estimate in Wh
float SimpleBattery::availablePowerW() const
{
    return voltage() * m_remainingAh;
}
}  // namespace lotusim::gazebo