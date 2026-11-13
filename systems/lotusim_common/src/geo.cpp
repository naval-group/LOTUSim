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

#include <cmath>

namespace lotusim::common::geo {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

using gz::math::SphericalCoordinates;

// TODO: a non-zero <heading_deg> is ignored for now
SphericalCoordinates withoutHeadingOffset(const SphericalCoordinates& origin)
{
    SphericalCoordinates copy = origin;
    copy.SetHeadingOffset(gz::math::Angle(0.0));
    return copy;
}

}  // namespace

double wrapDegrees(double degrees)
{
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    // fmod of a tiny negative angle can round up to exactly 360; + 0.0
    // turns -0 into 0.
    return wrapped >= 360.0 ? 0.0 : wrapped + 0.0;
}

double headingToYaw(double heading_deg)
{
    return (90.0 - heading_deg) * kDegToRad;
}

double yawToHeading(double yaw_rad)
{
    return wrapDegrees(90.0 - yaw_rad / kDegToRad);
}

double courseOf(double east_mps, double north_mps)
{
    if (std::hypot(east_mps, north_mps) < kMinCourseSpeed) {
        return kCourseNotAvailable;
    }
    return wrapDegrees(std::atan2(east_mps, north_mps) / kDegToRad);
}

float toFloatDegrees(double degrees)
{
    const float f = static_cast<float>(wrapDegrees(degrees));
    return f >= 360.0f ? 0.0f : f;
}

gz::math::Vector3d geoToLocal(
    const SphericalCoordinates& world_origin,
    double latitude_deg,
    double longitude_deg,
    double altitude_m)
{
    const SphericalCoordinates origin = withoutHeadingOffset(world_origin);
    // The point on the reference surface, projected onto the tangent plane.
    const auto on_plane = origin.PositionTransform(
        {latitude_deg * kDegToRad,
         longitude_deg * kDegToRad,
         origin.ElevationReference()},
        SphericalCoordinates::SPHERICAL,
        SphericalCoordinates::LOCAL2);
    return {on_plane.X(), on_plane.Y(), altitude_m};
}

gz::math::Vector3d localToGeo(
    const SphericalCoordinates& world_origin,
    const gz::math::Vector3d& local)
{
    const SphericalCoordinates origin = withoutHeadingOffset(world_origin);
    // Reference surface point at (x, y) with the reference's elevation.
    // since elevation grows almost linearly with z, a few iterations reduce
    // the error to micrometre precision, even at 1000 km
    double z = 0.0;
    gz::math::Vector3d spherical;
    for (int i = 0; i < 8; ++i) {
        spherical = origin.PositionTransform(
            {local.X(), local.Y(), z},
            SphericalCoordinates::LOCAL2,
            SphericalCoordinates::SPHERICAL);
        const double error = spherical.Z() - origin.ElevationReference();
        if (std::abs(error) < 1e-6) {
            break;
        }
        z -= error;
    }
    return {spherical.X() / kDegToRad, spherical.Y() / kDegToRad, local.Z()};
}

}  // namespace lotusim::common::geo
