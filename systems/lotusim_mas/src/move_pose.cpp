/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_mas/move_pose.hpp"

#include "lotusim_common/geo.hpp"

namespace lotusim::mas {

std::optional<gz::math::Pose3d> resolveMovePose(
    const lotusim_msgs::msg::MASCmd& msg,
    const LatLongToLocal& to_local)
{
    const auto xyz = to_local(
        msg.geo_point.latitude,
        msg.geo_point.longitude,
        msg.geo_point.altitude);
    if (!xyz) {
        return std::nullopt;
    }
    return gz::math::Pose3d(
        std::get<0>(*xyz),
        std::get<1>(*xyz),
        std::get<2>(*xyz),
        0.0,
        0.0,
        common::geo::headingToYaw(msg.heading));
}

}  // namespace lotusim::mas
