/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "waypoint_follower/control_step.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

using lotusim::gazebo::controlStepSeconds;
using namespace std::chrono_literals;

namespace {

gz::sim::UpdateInfo info(std::chrono::steady_clock::duration dt, bool paused)
{
    gz::sim::UpdateInfo update;
    update.dt = dt;
    update.paused = paused;
    return update;
}

}  // namespace

TEST(ControlStep, RunningStepInSeconds)
{
    EXPECT_DOUBLE_EQ(*controlStepSeconds(info(200ms, false)), 0.2);
}

TEST(ControlStep, PausedDoesNotRun)
{
    EXPECT_FALSE(controlStepSeconds(info(0ms, true)));
    // Even if a paused update reported a duration.
    EXPECT_FALSE(controlStepSeconds(info(200ms, true)));
}

TEST(ControlStep, ZeroStepDoesNotRun)
{
    EXPECT_FALSE(controlStepSeconds(info(0ms, false)));
}

TEST(ControlStep, SubMillisecondStepIsKept)
{
    // A cast to whole milliseconds used to make this 0.
    EXPECT_DOUBLE_EQ(*controlStepSeconds(info(500us, false)), 0.0005);
    EXPECT_DOUBLE_EQ(*controlStepSeconds(info(1500us, false)), 0.0015);
}

// gz-sim's dependency chain can crash during static teardown (see
// physics_engine_interface's tests), so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
