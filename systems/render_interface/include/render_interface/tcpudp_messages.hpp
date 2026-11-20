/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_TCPUDP_MESSAGES_HH_
#define LOTUSIM_TCPUDP_MESSAGES_HH_

/**
 * @brief The JSON TcpUdpInterface sends to the renderer: commands over TCP,
 * the world state over UDP, laid out as lotusim_msgs/RendererState.
 *
 * Positions are geographic: "latitude" and "longitude" in WGS84 degrees,
 * "altitude" in metres from sea level. Rotations are the ENU world-frame
 * attitude quaternion.
 *
 * Kept apart from the socket code so they can be tested: the format strings
 * hold literal JSON braces, which fmt reads as replacement fields unless they
 * are doubled. An undoubled brace throws fmt::format_error at run time under
 * C++17, from the Gazebo thread, which terminates the server; FMT_STRING turns
 * that into a compile error.
 */

#include <chrono>
#include <string>
#include <vector>

#include "lotusim_common/geo_types.hpp"

namespace lotusim::gazebo {

using lotusim::common::GeoPose;

std::string formatCreateMessage(
    const std::string& name,
    const GeoPose& pose,
    const std::string& type);

std::string formatDestroyMessage(const std::string& name);

std::string formatExplodeMessage(const std::string& name);

/** @brief One vessel of the world state, as lotusim_msgs/RendererVessel. */
struct RendererVessel {
    std::string vessel_name;
    std::string renderer_obj_name;
    GeoPose vessel_pose;
};

/**
 * @brief The UDP world state, as lotusim_msgs/RendererState:
 *
 * {"header": {"stamp": {"sec", "nanosec"}, "frame_id": "world"},
 *  "vessels": [{"vessel_name", "renderer_obj_name",
 *               "vessel_pose": {"position": {"latitude", "longitude",
 *                                            "altitude"},
 *                               "orientation": {"x", "y", "z", "w"}}}]}
 *
 * @param runTime Simulation time of the snapshot
 * @param vessels Every vessel alive; an empty array removes them all
 */
std::string formatStateMessage(
    const std::chrono::steady_clock::duration& runTime,
    const std::vector<RendererVessel>& vessels);

}  // namespace lotusim::gazebo

#endif
