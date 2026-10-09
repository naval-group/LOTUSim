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

#include <gtest/gtest.h>

using lotusim::gazebo::toXdynCommands;
using nlohmann::json;

TEST(ToXdynCommands, ConvertsRpmToRadPerSecond)
{
    const json out = toXdynCommands({{"propeller(rpm)", 60.0}});
    EXPECT_NEAR(out["propeller(rpm)"].get<double>(), 2.0 * 3.14159265358979, 1e-9);
}

TEST(ToXdynCommands, ConvertsEveryPropeller)
{
    const json out = toXdynCommands(
        {{"PSPropRudd(rpm)", 200}, {"SBPropRudd(rpm)", 200}});
    EXPECT_NEAR(out["PSPropRudd(rpm)"].get<double>(), 20.943951, 1e-6);
    EXPECT_NEAR(out["SBPropRudd(rpm)"].get<double>(), 20.943951, 1e-6);
}

TEST(ToXdynCommands, LeavesOtherSignalsUnchanged)
{
    const json in = {
        {"propeller(P/D)", 0.88},
        {"propeller(beta)", 0.1},
        {"rudder(angle)", -0.2}};
    EXPECT_EQ(toXdynCommands(in), in);
}

TEST(ToXdynCommands, IgnoresNonNumericRpmAndNonObjects)
{
    const json in = {{"propeller(rpm)", "fast"}};
    EXPECT_EQ(toXdynCommands(in), in);
    EXPECT_EQ(toXdynCommands(json::array({1, 2})), json::array({1, 2}));
}

TEST(ToXdynCommands, DoesNotModifyItsInput)
{
    const json in = {{"propeller(rpm)", 200.0}};
    toXdynCommands(in);
    EXPECT_EQ(in["propeller(rpm)"].get<double>(), 200.0);
}
