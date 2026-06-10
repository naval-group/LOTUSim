/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <string>
#include <unordered_map>
#include <stdexcept>

namespace lotusim::gazebo{
/**
 * @brief energy density of supported fuel types in J/L
 *
 * used by Generator subclasses to compute fuel consumption:
 *   fuel_consumed = (power_w * dt) / (efficiency * energy_density_j_per_l)
 *
 * To add a new fuel type, add an entry here -> no code changes needed anywhere else
 */
inline const std::unordered_map<std::string, float> kFuelEnergyDensityJPerL = {
    { "diesel",   34'920'000.0f },  // ~9.7  kWh/L
    { "gasoline", 32'400'000.0f },  // ~9.0  kWh/L
    { "lng",      22'320'000.0f },  // ~6.2  kWh/L  liquefied natural gas
    { "hydrogen",  9'000'000.0f },  // ~2.5  kWh/L  compressed at 700 bar
    { "methanol", 15'840'000.0f },  // ~4.4  kWh/L
};

/**
 * @brief returns energy density in J/L for the given fuel type
 */
inline float getFuelEnergyDensity(const std::string& fuel_type)
{
    const auto it = kFuelEnergyDensityJPerL.find(fuel_type);
    if (it == kFuelEnergyDensityJPerL.end()) {
        throw std::invalid_argument(
            "Unknown fuel type: '" + fuel_type + "'. "
            "Add it to fuel_properties.hpp");
    }
    return it->second;
}    
} //namespace lotusim::gazebo