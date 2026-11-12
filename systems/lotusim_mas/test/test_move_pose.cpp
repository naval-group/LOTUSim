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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>

using lotusim::mas::LatLongToLocal;
using lotusim::mas::resolveMovePose;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A stand-in for the world's projection: 100 km per degree, z the altitude.
const LatLongToLocal kToLocal = [](double lat, double lon, double alt) {
    return std::make_optional(std::make_tuple(lon * 1e5, lat * 1e5, alt));
};

lotusim_msgs::msg::MASCmd command(
    double lat,
    double lon,
    double alt,
    double heading)
{
    lotusim_msgs::msg::MASCmd msg;
    msg.cmd_type = lotusim_msgs::msg::MASCmd::MOVE_CMD;
    msg.vessel_name = "ship";
    msg.geo_point.latitude = lat;
    msg.geo_point.longitude = lon;
    msg.geo_point.altitude = alt;
    msg.heading = static_cast<float>(heading);
    return msg;
}

}  // namespace

TEST(MovePose, PositionFromTheGeoPoint)
{
    const auto pose = resolveMovePose(command(0.01, 0.02, -30.0, 0.0), kToLocal);
    ASSERT_TRUE(pose);
    EXPECT_NEAR(pose->X(), 2000.0, 1e-6);
    EXPECT_NEAR(pose->Y(), 1000.0, 1e-6);
    EXPECT_EQ(pose->Z(), -30.0);
}

TEST(MovePose, ZeroesAreValues)
{
    // Latitude 0, longitude 0, the surface and North: before, a command with
    // only zeroes kept the previous pose.
    const auto pose = resolveMovePose(command(0.0, 0.0, 0.0, 0.0), kToLocal);
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->Pos(), gz::math::Vector3d::Zero);
    EXPECT_NEAR(pose->Rot().Yaw(), kPi / 2.0, 1e-6);  // bow to the North
}

TEST(MovePose, HeadingFromNorthClockwise)
{
    for (const auto& [heading, yaw] :
         {std::pair{0.0, kPi / 2.0},
          std::pair{90.0, 0.0},
          std::pair{180.0, -kPi / 2.0},
          std::pair{30.0, kPi / 3.0}}) {
        const auto pose =
            resolveMovePose(command(0.0, 0.0, 0.0, heading), kToLocal);
        ASSERT_TRUE(pose);
        EXPECT_NEAR(pose->Rot().Yaw(), yaw, 1e-6) << heading << " deg";
        EXPECT_NEAR(pose->Rot().Roll(), 0.0, 1e-9);
        EXPECT_NEAR(pose->Rot().Pitch(), 0.0, 1e-9);
    }
    const auto west = resolveMovePose(command(0.0, 0.0, 0.0, 270.0), kToLocal);
    ASSERT_TRUE(west);
    EXPECT_NEAR(std::abs(west->Rot().Yaw()), kPi, 1e-6);
}

TEST(MovePose, NoGeographicOrigin)
{
    const LatLongToLocal none = [](double, double, double) {
        return std::optional<std::tuple<double, double, double>>{};
    };
    EXPECT_FALSE(resolveMovePose(command(1.0, 2.0, 0.0, 90.0), none));
}

// Custom main instead of gtest_main, as the other packages' tests: gz-sim's
// dependency chain can segfault during static teardown; _Exit() skips it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
