/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/simple_generator.hpp"
#include "power_subsystem/generator.hpp"
#include "power_subsystem/fuel_properties.hpp"
#include <algorithm>
#include "lotusim_common/common.hpp"

// SimpleGenerator: fuel consumption is driven by bus current (currentA)

namespace lotusim::gazebo
{
void SimpleGenerator::receiveLoad(float currentA, float dt){
    if (isDepleted()){
        m_logger->warn(
            "SimpleGenerator [{}]: no fuel remaining, cannot supply load",
            Generator::name());
        return;
    }

    // power demand
    const float powerW = currentA * m_voltage_nominal;

    // fuel consumption model 
    // diesel energy density ≈ 9.7 kWh/L → 34_920_000 J/L
    const float energyDensity = getFuelEnergyDensity(m_fuel_type);
    const float fuel_consumed = (powerW * dt) / (m_efficiency * energyDensity);

    m_fuel_level = std::max(0.0f, m_fuel_level - fuel_consumed);

    m_logger->info(
            "SimpleGenerator [{}]: fuel level: {}",
            Generator::name(), m_fuel_level);

    m_logger->debug(
        "SimpleGenerator [{}]: load={:.3f} A power={:.1f} W "
        "fuel_consumed={:.6f} L remaining={:.3f}/{:.3f} L ratio={:.3f}",
        Generator::name(), currentA, powerW,
        fuel_consumed, m_fuel_level, m_fuel_capacity, fuelRatio());

    if (isDepleted()) {
        m_logger->warn("SimpleGenerator [{}]: fuel depleted", Generator::name());
    }
}
} //namespace lotusim::gazebo