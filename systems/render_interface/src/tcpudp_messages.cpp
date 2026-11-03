/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "render_interface/tcpudp_messages.hpp"

#include <fmt/format.h>

namespace lotusim::gazebo {

std::string formatCreateMessage(
    const std::string& name,
    const gz::math::Pose3d& pose,
    const std::string& type)
{
    return fmt::format(
        FMT_STRING(R"({{
        "cmd": "create",
        "name": "{}",
        "pose": {{
            "x": {},
            "y": {},
            "z": {}
        }},
        "type": "{}"
    }})"),
        name,
        pose.X(),
        pose.Y(),
        pose.Z(),
        type);
}

std::string formatDestroyMessage(const std::string& name)
{
    return fmt::format(
        FMT_STRING(R"(
        {{
            "cmd": "destroy",
            "name": "{}"
        }})"),
        name);
}

std::string formatExplodeMessage(const std::string& name)
{
    return fmt::format(
        FMT_STRING(R"(
        {{
            "cmd": "explode",
            "name": "{}"
        }})"),
        name);
}

std::string formatPositionMessage(
    const std::string& name,
    const gz::math::Pose3d& pose)
{
    return fmt::format(
        FMT_STRING(R"(
        {{
            "name": "{}",
            "position": {{
                "x": {},
                "y": {},
                "z": {}
            }},
            "rotation": {{
                "x": {},
                "y": {},
                "z": {},
                "w": {}
            }},
            "thrusters": [
                    ]}})"),
        name,
        pose.X(),
        pose.Y(),
        pose.Z(),
        pose.Rot().X(),
        pose.Rot().Y(),
        pose.Rot().Z(),
        pose.Rot().W());
}

}  // namespace lotusim::gazebo
