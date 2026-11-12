/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_WAYPOINT_FOLLOWER_CONTROL_STEP_HH_
#define LOTUSIM_WAYPOINT_FOLLOWER_CONTROL_STEP_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <optional>

namespace lotusim::gazebo {

/**
 * @brief The duration the follower integrates over this update, in seconds,
 * or nothing when it must not run.
 *
 * The controller divides by it (derivative terms) and multiplies velocities
 * by it. Gazebo keeps calling Update while paused, with dt = 0: dividing by
 * that turned the derivative into 0/0 = NaN on the second paused step, and
 * the NaN never left the vessel's velocity and pose. The duration is kept
 * fractional: a cast to whole milliseconds made steps under 1 ms zero too.
 */
inline std::optional<double> controlStepSeconds(const gz::sim::UpdateInfo& info)
{
    if (info.paused) {
        return std::nullopt;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!(dt > 0.0)) {
        return std::nullopt;
    }
    return dt;
}

}  // namespace lotusim::gazebo

#endif
