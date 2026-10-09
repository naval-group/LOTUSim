/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/xdyn_commands.hpp"

#include <string>

namespace lotusim::gazebo {

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double rpm_to_rad_per_s = 2.0 * pi / 60.0;

bool isRpmSignal(const std::string& signal)
{
    static const std::string suffix = "(rpm)";
    return signal.size() >= suffix.size() &&
           signal.compare(signal.size() - suffix.size(), suffix.size(), suffix) ==
               0;
}

}  // namespace

nlohmann::json toXdynCommands(const nlohmann::json& commands)
{
    nlohmann::json converted = commands;
    if (!converted.is_object()) {
        return converted;
    }
    for (auto& [signal, value] : converted.items()) {
        if (value.is_number() && isRpmSignal(signal)) {
            value = value.get<double>() * rpm_to_rad_per_s;
        }
    }
    return converted;
}

}  // namespace lotusim::gazebo
