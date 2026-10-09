/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_XDYN_COMMANDS_HH_
#define LOTUSIM_XDYN_COMMANDS_HH_

/**
 * @brief Converts LOTUSim's actuator commands to the units xdyn expects.
 *
 * xdyn's co-simulation API takes command values in SI units, with no unit
 * attached: a propeller's "(rpm)" signal is read in rad/s, whatever its name
 * says. LOTUSim's API speaks revolutions per minute — the examples, the
 * VesselCmd strings and the power subsystem's /rpm topics all do — so the
 * conversion happens once, on the way out to xdyn. Sending rpm unconverted
 * spun propellers 60/2π ≈ 9.55 times too fast (an LRAUV at 200 rpm cruised at
 * 9.2 m/s instead of 1.1 m/s).
 */

#include <nlohmann/json.hpp>

namespace lotusim::gazebo {

/**
 * @brief Returns @p commands with every numeric "<actuator>(rpm)" value
 * converted from revolutions per minute to rad/s. Other signals (P/D, beta,
 * angles) pass through unchanged.
 */
nlohmann::json toXdynCommands(const nlohmann::json& commands);

}  // namespace lotusim::gazebo

#endif
