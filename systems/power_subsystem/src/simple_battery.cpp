/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/simple_battery.hpp"
#include <algorithm>
#include "lotusim_common/common.hpp" 

namespace lotusim::gazebo
{
Battery::~Battery() = default;

void SimpleBattery::receiveLoad(float currentA, float dt)
{
    m_remainingAh -= currentA * dt;
    m_remainingAh  = std::max(m_remainingAh, 0.0f);
    updateVoltage();
}

void SimpleBattery::updateVoltage()
{
    // voltage scales linearly with SOC between voltage_min and voltage_nominal
    // this is a simplification
    m_voltage = m_voltageMin + getStateOfCharge() * (m_voltage_nominal - m_voltageMin);
}

float SimpleBattery::voltage() const
{
    return m_voltage;
}

void SimpleBattery::receiveCharge(float currentA, float dt)
{
    m_remainingAh += currentA * dt;
    m_remainingAh  = std::min(m_remainingAh, m_capacityAh);
    updateVoltage();
}

float SimpleBattery::getStateOfCharge() const
{
    if (m_capacityAh <= 0.0f) { return 0.0f; }
    return std::clamp(m_remainingAh / m_capacityAh, 0.0f, 1.0f);
}

// for now - energy estimate in Wh 
float SimpleBattery::availablePowerW() const
{
    return voltage() * m_remainingAh;
}
} // namespace lotusim::gazebo