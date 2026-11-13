/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_common/geo.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>

using namespace lotusim::common::geo;
using gz::math::SphericalCoordinates;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A reference point in the Singapore Strait
SphericalCoordinates worldOrigin(double elevation = 0.0, double heading_deg = 0.0)
{
    return SphericalCoordinates(
        SphericalCoordinates::EARTH_WGS84,
        gz::math::Angle(1.26 * kPi / 180.0),
        gz::math::Angle(103.75 * kPi / 180.0),
        elevation,
        gz::math::Angle(heading_deg * kPi / 180.0));
}

// Gazebo's own conversion of a WGS84 point to the tangent plane.
gz::math::Vector3d gazeboLocal(
    const SphericalCoordinates& origin,
    double lat_deg,
    double lon_deg,
    double alt)
{
    return origin.PositionTransform(
        {lat_deg * kPi / 180.0, lon_deg * kPi / 180.0, alt},
        SphericalCoordinates::SPHERICAL,
        SphericalCoordinates::LOCAL2);
}

}  // namespace

TEST(GeoAngles, HeadingAndYaw)
{
    EXPECT_NEAR(headingToYaw(0.0), kPi / 2.0, 1e-12);   // North
    EXPECT_NEAR(headingToYaw(90.0), 0.0, 1e-12);        // East
    EXPECT_NEAR(headingToYaw(180.0), -kPi / 2.0, 1e-12);
    EXPECT_NEAR(yawToHeading(kPi / 2.0), 0.0, 1e-9);
    EXPECT_NEAR(yawToHeading(0.0), 90.0, 1e-9);
    EXPECT_NEAR(yawToHeading(kPi), 270.0, 1e-9);
    EXPECT_NEAR(yawToHeading(-kPi / 2.0), 180.0, 1e-9);
    for (double heading = 0.0; heading < 360.0; heading += 15.0) {
        EXPECT_NEAR(yawToHeading(headingToYaw(heading)), heading, 1e-9);
    }
}

TEST(GeoAngles, WrapDegrees)
{
    EXPECT_EQ(wrapDegrees(360.0), 0.0);
    EXPECT_EQ(wrapDegrees(-90.0), 270.0);
    EXPECT_EQ(wrapDegrees(725.0), 5.0);
    EXPECT_FALSE(std::signbit(wrapDegrees(-0.0)));
    EXPECT_LT(wrapDegrees(-1e-17), 360.0);
}

TEST(GeoAngles, Course)
{
    EXPECT_NEAR(courseOf(0.0, 2.0), 0.0, 1e-9);
    EXPECT_NEAR(courseOf(2.0, 0.0), 90.0, 1e-9);
    EXPECT_NEAR(courseOf(0.0, -2.0), 180.0, 1e-9);
    EXPECT_NEAR(courseOf(-2.0, 0.0), 270.0, 1e-9);
    EXPECT_EQ(courseOf(0.01, 0.01), 360.0);
}

TEST(GeoProjection, OriginIsZero)
{
    const auto origin = worldOrigin();
    const auto local = geoToLocal(origin, 1.26, 103.75, -30.0);
    EXPECT_NEAR(local.X(), 0.0, 1e-6);
    EXPECT_NEAR(local.Y(), 0.0, 1e-6);
    EXPECT_EQ(local.Z(), -30.0);
}

TEST(GeoProjection, ZIsTheAltitudeFarFromTheOrigin)
{
    // About 100 km North-East: Gazebo's point is some 785 m below the
    // plane for a vessel at the surface; the projection keeps it at z = 0,
    // with the same x and y.
    const auto origin = worldOrigin();
    const double lat = 1.26 + 0.636;
    const double lon = 103.75 + 0.636;
    const auto gazebo = gazeboLocal(origin, lat, lon, 0.0);
    ASSERT_LT(gazebo.Z(), -700.0);

    const auto local = geoToLocal(origin, lat, lon, 0.0);
    EXPECT_EQ(local.Z(), 0.0);
    EXPECT_NEAR(local.X(), gazebo.X(), 1e-6);
    EXPECT_NEAR(local.Y(), gazebo.Y(), 1e-6);
    EXPECT_NEAR(std::hypot(local.X(), local.Y()), 100e3, 1e3);

    EXPECT_EQ(geoToLocal(origin, lat, lon, -12.5).Z(), -12.5);
    EXPECT_EQ(geoToLocal(origin, lat, lon, 300.0).Z(), 300.0);
}

TEST(GeoProjection, RoundTrip)
{
    const auto origin = worldOrigin();
    for (double distance : {0.0, 10.0, 1e3, 100e3, 1000e3}) {
        for (double bearing = 0.0; bearing < 360.0; bearing += 45.0) {
            const double b = bearing * kPi / 180.0;
            for (double altitude : {-30.0, 0.0, 120.0}) {
                const gz::math::Vector3d local(
                    distance * std::sin(b),
                    distance * std::cos(b),
                    altitude);
                const auto geo = localToGeo(origin, local);
                EXPECT_EQ(geo.Z(), altitude);
                const auto back =
                    geoToLocal(origin, geo.X(), geo.Y(), geo.Z());
                EXPECT_NEAR(back.X(), local.X(), 1e-5)
                    << distance << " m at " << bearing << " deg";
                EXPECT_NEAR(back.Y(), local.Y(), 1e-5)
                    << distance << " m at " << bearing << " deg";
                EXPECT_EQ(back.Z(), altitude);
            }
        }
    }
}

TEST(GeoProjection, NorthIsUp)
{
    // 1 km North of the origin: latitude grows, longitude does not.
    const auto origin = worldOrigin();
    const auto geo = localToGeo(origin, {0.0, 1000.0, 0.0});
    EXPECT_GT(geo.X(), 1.26 + 0.008);
    EXPECT_LT(geo.X(), 1.26 + 0.010);
    EXPECT_NEAR(geo.Y(), 103.75, 1e-9);
    // 1 km East: longitude grows.
    const auto east = localToGeo(origin, {1000.0, 0.0, 0.0});
    EXPECT_GT(east.Y(), 103.75 + 0.008);
    EXPECT_NEAR(east.X(), 1.26, 1e-6);
}

TEST(GeoAngles, FloatDegStayBelow360)
{
    EXPECT_EQ(toFloatDegrees(359.99999), 0.0f);  // casts to 360.0f else
    EXPECT_EQ(toFloatDegrees(90.0), 90.0f);
    // moving vessel heading just west of North is not "not available"
    const double course = courseOf(-1e-9, 1.0);
    EXPECT_LT(course, 360.0);
    EXPECT_EQ(toFloatDegrees(course), 0.0f);
}

TEST(GeoProjection, ElevationRefDoesNotShiftZ)
{
    const auto origin = worldOrigin(150.0);
    const auto local = geoToLocal(origin, 1.26, 103.75, 0.0);
    EXPECT_NEAR(local.X(), 0.0, 1e-6);
    EXPECT_NEAR(local.Y(), 0.0, 1e-6);
    EXPECT_EQ(local.Z(), 0.0);
    const auto geo = localToGeo(origin, local);
    EXPECT_NEAR(geo.X(), 1.26, 1e-9);
    EXPECT_EQ(geo.Z(), 0.0);
}

TEST(GeoProjection, RoundTripWithOtherOrigins)
{
    for (double elevation : {0.0, 150.0}) {
        for (double heading : {0.0, 90.0, 217.0}) {
            const auto origin = worldOrigin(elevation, heading);
            const gz::math::Vector3d local(30e3, -40e3, -20.0);
            const auto geo = localToGeo(origin, local);
            const auto back = geoToLocal(origin, geo.X(), geo.Y(), geo.Z());
            EXPECT_NEAR(back.X(), local.X(), 1e-5)
                << "elevation " << elevation << " heading " << heading;
            EXPECT_NEAR(back.Y(), local.Y(), 1e-5);
            EXPECT_EQ(back.Z(), -20.0);
        }
    }
}

TEST(GeoProjection, HeadingOffsetIsIgnored)
{
    const auto origin = worldOrigin(0.0, 90.0);
    const auto geo = localToGeo(origin, {0.0, 1000.0, 0.0});
    EXPECT_GT(geo.X(), 1.26 + 0.008);
    EXPECT_NEAR(geo.Y(), 103.75, 1e-9);
}

// Custom main instead of gtest_main: gz-sim8 segfaults during static teardown 
// in the nix build sandbox. _Exit() skips teardown; the OS reclaims resources on
// process exit
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
