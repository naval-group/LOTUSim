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
 * one vessel of the pose update over UDP.
 *
 * Kept apart from the socket code so they can be tested: the format strings
 * hold literal JSON braces, which fmt reads as replacement fields unless they
 * are doubled. An undoubled brace throws fmt::format_error at run time under
 * C++17, from the Gazebo thread, which terminates the server; FMT_STRING turns
 * that into a compile error.
 */

#include <gz/math/Pose3.hh>
#include <string>

namespace lotusim::gazebo {

std::string formatCreateMessage(
    const std::string& name,
    const gz::math::Pose3d& pose,
    const std::string& type);

std::string formatDestroyMessage(const std::string& name);

std::string formatExplodeMessage(const std::string& name);

/** @brief One vessel's entry in the UDP pose update's "VesselsInfo" array. */
std::string formatPositionMessage(
    const std::string& name,
    const gz::math::Pose3d& pose);

}  // namespace lotusim::gazebo

#endif
