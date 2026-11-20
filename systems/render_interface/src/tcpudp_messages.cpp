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
    const GeoPose& pose,
    const std::string& type)
{
    return fmt::format(
        FMT_STRING(R"({{
        "cmd": "create",
        "name": "{}",
        "pose": {{
            "latitude": {},
            "longitude": {},
            "altitude": {}
        }},
        "type": "{}"
    }})"),
        name,
        pose.position.latitude,
        pose.position.longitude,
        pose.position.altitude,
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

std::string formatStateMessage(
    const std::chrono::steady_clock::duration& runTime,
    const std::vector<RendererVessel>& vessels)
{
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(runTime).count();
    std::string msg = fmt::format(
        FMT_STRING(
            R"({{"header":{{"stamp":{{"sec":{},"nanosec":{}}},"frame_id":"world"}},"vessels":[)"),
        ns / 1000000000,
        ns % 1000000000);
    bool first = true;
    for (const auto& vessel : vessels) {
        if (!first) {
            msg += ',';
        }
        first = false;
        const GeoPose& pose = vessel.vessel_pose;
        msg += fmt::format(
            // No whitespace: the state goes out as one UDP datagram, so
            // every byte per vessel lowers the vessel count it can carry.
            FMT_STRING(
                R"({{"vessel_name":"{}","renderer_obj_name":"{}","vessel_pose":{{"position":{{"latitude":{},"longitude":{},"altitude":{}}},"orientation":{{"x":{},"y":{},"z":{},"w":{}}}}}}})"),
            vessel.vessel_name,
            vessel.renderer_obj_name,
            pose.position.latitude,
            pose.position.longitude,
            pose.position.altitude,
            pose.orientation.X(),
            pose.orientation.Y(),
            pose.orientation.Z(),
            pose.orientation.W());
    }
    msg += "]}";
    return msg;
}

}  // namespace lotusim::gazebo
