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
    GeoPose pose;
    pose.position.latitude = 43.123456789;
    pose.position.longitude = -5.987654321;
    pose.position.altitude = -3.25;
    json msg;
    ASSERT_NO_THROW(
        msg = json::parse(formatCreateMessage("boat", pose, "fremm")));
    EXPECT_EQ(msg["cmd"], "create");
    EXPECT_EQ(msg["name"], "boat");
    EXPECT_EQ(msg["type"], "fremm");
    // Full precision: at 1e-9 deg, a vessel is placed to about 0.1 mm.
    EXPECT_DOUBLE_EQ(msg["pose"]["latitude"].get<double>(), 43.123456789);
    EXPECT_DOUBLE_EQ(msg["pose"]["longitude"].get<double>(), -5.987654321);
    EXPECT_DOUBLE_EQ(msg["pose"]["altitude"].get<double>(), -3.25);
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

TEST(TcpUdpMessages, StateIsValidJson)
{
    GeoPose pose;
    pose.position.latitude = 1.5;
    pose.position.longitude = -2.0;
    pose.position.altitude = 3.25;
    pose.orientation = gz::math::Quaterniond(0.5, 0.5, 0.5, 0.5);  // w x y z
    const std::vector<RendererVessel> vessels{
        {"boat", "fremm", pose},
        {"drone", "", GeoPose{}}};
    json msg;
    ASSERT_NO_THROW(
        msg = json::parse(formatStateMessage(
            std::chrono::seconds(12) + std::chrono::nanoseconds(345),
            vessels)));
    EXPECT_EQ(msg["header"]["stamp"]["sec"], 12);
    EXPECT_EQ(msg["header"]["stamp"]["nanosec"], 345);
    EXPECT_EQ(msg["header"]["frame_id"], "world");
    ASSERT_EQ(msg["vessels"].size(), 2u);
    const auto& boat = msg["vessels"][0];
    EXPECT_EQ(boat["vessel_name"], "boat");
    EXPECT_EQ(boat["renderer_obj_name"], "fremm");
    const auto& p = boat["vessel_pose"];
    EXPECT_DOUBLE_EQ(p["position"]["latitude"].get<double>(), 1.5);
    EXPECT_DOUBLE_EQ(p["position"]["longitude"].get<double>(), -2.0);
    EXPECT_DOUBLE_EQ(p["position"]["altitude"].get<double>(), 3.25);
    EXPECT_DOUBLE_EQ(p["orientation"]["x"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(p["orientation"]["y"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(p["orientation"]["z"].get<double>(), 0.5);
    EXPECT_DOUBLE_EQ(p["orientation"]["w"].get<double>(), 0.5);
    EXPECT_EQ(msg["vessels"][1]["vessel_name"], "drone");
}

TEST(TcpUdpMessages, EmptyStateIsValidJson)
{
    json msg;
    ASSERT_NO_THROW(
        msg = json::parse(formatStateMessage(
            std::chrono::steady_clock::duration::zero(), {})));
    EXPECT_TRUE(msg["vessels"].is_array());
    EXPECT_TRUE(msg["vessels"].empty());
}
