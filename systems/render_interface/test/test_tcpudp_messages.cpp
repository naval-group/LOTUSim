/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "render_interface/tcpudp_messages.hpp"

using namespace lotusim::gazebo;
using nlohmann::json;

TEST(TcpUdpMessages, CreateIsValidJson)
{
    const gz::math::Pose3d pose(1.5, -2.0, 3.25, 0, 0, 0);
    json msg;
    ASSERT_NO_THROW(
        msg = json::parse(formatCreateMessage("boat", pose, "fremm")));
    EXPECT_EQ(msg["cmd"], "create");
    EXPECT_EQ(msg["name"], "boat");
    EXPECT_EQ(msg["type"], "fremm");
    EXPECT_DOUBLE_EQ(msg["pose"]["x"].get<double>(), 1.5);
    EXPECT_DOUBLE_EQ(msg["pose"]["y"].get<double>(), -2.0);
    EXPECT_DOUBLE_EQ(msg["pose"]["z"].get<double>(), 3.25);
}

TEST(TcpUdpMessages, DestroyIsValidJson)
{
    json msg;
    ASSERT_NO_THROW(msg = json::parse(formatDestroyMessage("boat")));
    EXPECT_EQ(msg["cmd"], "destroy");
    EXPECT_EQ(msg["name"], "boat");
}

TEST(TcpUdpMessages, ExplodeIsValidJson)
{
    json msg;
    ASSERT_NO_THROW(msg = json::parse(formatExplodeMessage("boat")));
    EXPECT_EQ(msg["cmd"], "explode");
    EXPECT_EQ(msg["name"], "boat");
}

TEST(TcpUdpMessages, PositionIsValidJson)
{
    const gz::math::Pose3d pose(
        1.5,
        -2.0,
        3.25,
        0.5,
        0.5,
        0.5,
        0.5);  // w x y z
    json msg;
    ASSERT_NO_THROW(msg = json::parse(formatPositionMessage("boat", pose)));
    EXPECT_EQ(msg["name"], "boat");
    EXPECT_DOUBLE_EQ(msg["position"]["x"].get<double>(), 1.5);
    EXPECT_DOUBLE_EQ(msg["position"]["y"].get<double>(), -2.0);
    EXPECT_DOUBLE_EQ(msg["position"]["z"].get<double>(), 3.25);
    EXPECT_DOUBLE_EQ(msg["rotation"]["x"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(msg["rotation"]["y"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(msg["rotation"]["z"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(msg["rotation"]["w"].get<double>(), 0.5);
    EXPECT_TRUE(msg["thrusters"].is_array());
    EXPECT_TRUE(msg["thrusters"].empty());
}
