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
        return 360.0;
    }
    return wrapDegrees(std::atan2(east_mps, north_mps) / kDegToRad);
}

gz::math::Vector3d geoToLocal(
    const SphericalCoordinates& origin,
    double latitude_deg,
    double longitude_deg,
    double altitude_m)
{
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
    const SphericalCoordinates& origin,
    const gz::math::Vector3d& local)
{
    // The point of the reference surface whose projection is (x, y): on the
    // origin's vertical through (x, y), where the elevation is the
    // reference's. The elevation grows with z at a rate close to 1, so each
    // step removes almost all of the error; a few steps reach a micrometre
    // at 1000 km.
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
