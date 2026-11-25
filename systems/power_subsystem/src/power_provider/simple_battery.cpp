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
#include <cmath>

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
    // capacity_ah and initial_soc as Battery read them.
    , m_remainingAh(static_cast<double>(m_capacityAh) * m_initialSoc)
{
    // Voltage rises from voltage_min to voltage_nominal as the battery charges
    if (!(m_voltage_nominal > m_voltageMin)) {
        m_logger->error(
            "SimpleBattery [{}]: voltage_nominal must be > voltage_min (got {} "
            "and {}), using a constant voltage_min",
            m_provider_name,
            m_voltage_nominal,
            m_voltageMin);
        m_voltage_nominal = m_voltageMin;
    }
    updateVoltage();
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
    if (!(dt > 0.0f) || !std::isfinite(dt)) {
        return;
    }
    if (!std::isfinite(currentA)) {
        // debug: this runs every tick
        m_logger->debug(
            "SimpleBattery [{},{}]: non-finite current ignored",
            m_provider_name,
            m_vessel_name);
        return;
    }
    // currentA > 0 discharges, < 0 charges; dt in seconds and the charge is in Ah
    constexpr double seconds_per_hour = 3600.0;
    m_remainingAh = std::clamp(
        m_remainingAh - static_cast<double>(currentA) * dt / seconds_per_hour,
        0.0,
        static_cast<double>(m_capacityAh));
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
    // simplified linear voltage SOC between voltage_min and voltage_nominal
    m_voltage =
        m_voltageMin + getStateOfCharge() * (m_voltage_nominal - m_voltageMin);
}

float SimpleBattery::voltage() const
{
    return m_voltage;
}

float SimpleBattery::getStateOfCharge() const
{
    return std::clamp(
        static_cast<float>(m_remainingAh / m_capacityAh), 0.0f, 1.0f);
}

// for now - energy estimate in Wh
float SimpleBattery::availablePowerW() const
{
    return static_cast<float>(voltage() * m_remainingAh);
}
}  // namespace lotusim::gazebo