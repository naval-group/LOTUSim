/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_MAS_MOVE_POSE_HPP
#define LOTUSIM_MAS_MOVE_POSE_HPP

#include <functional>
#include <gz/math/Pose3.hh>
#include <optional>
#include <tuple>

#include "lotusim_msgs/msg/mas_cmd.hpp"

namespace lotusim::mas {

/// Local ENU position of a WGS84 point, or nothing if the world has no
/// geographic origin.
using LatLongToLocal =
    std::function<std::optional<std::tuple<double, double, double>>(
        double lat,
        double lon,
        double alt)>;

/**
 * @brief The pose a MASCmd asks for (create or move): its geo_point,
 * converted with to_local, and its heading, in degrees from true North,
 * clockwise, both always given. Nothing if the position cannot be converted.
 */
std::optional<gz::math::Pose3d> resolveMovePose(
    const lotusim_msgs::msg::MASCmd& msg,
    const LatLongToLocal& to_local);

}  // namespace lotusim::mas

#endif
