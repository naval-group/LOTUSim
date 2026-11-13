/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COMMON_GEO_HPP
#define LOTUSIM_COMMON_GEO_HPP

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

/**
 * @brief The one conversion step between what LOTUSim exposes and Gazebo's world.
 *
 * Exposed (MASCmd, VesselPosition, scenario files, AIS, GPS): WGS84 latitude
 * and longitude in degrees, altitude in metres from sea level, positive up
 * and negative when submerged, angles in degrees from true North, clockwise,
 * in [0, 360).
 *
 * Gazebo: a flat ENU world (x East, y North, z Up), the sea surface at
 * z = 0, yaw in radians from East, counter-clockwise.
 *
 * The flat world is a map projection: x and y are the projection of
 * latitude and longitude onto the plane tangent at the world's
 * <spherical_coordinates> origin, and z is the altitude. Both directions use
 * the same projection, so a position read back is the position given. Its
 * local scale error is about d^2 / 2R^2 at a distance d from the origin
 * (1e-4 at 100 km). Gazebo's own conversion instead returns the 3D point on
 * the tangent plane, whose z drops with the Earth's curvature (about 785 m
 * at 100 km): a vessel given at the surface ended up far below it.
 */
namespace lotusim::common::geo {

/// An angle in degrees brought into [0, 360).
double wrapDegrees(double degrees);

/// Heading (degrees from true North, clockwise) to ENU yaw (radians from East, counter-clockwise)
double headingToYaw(double heading_deg);

/// ENU yaw (radians from East, counter-clockwise) to heading (degrees from
/// true North, clockwise, in [0, 360)).
double yawToHeading(double yaw_rad);

/// course()' value when it's not defined (AIS's "not available")
constexpr double kCourseNotAvailable = 360.0;

/// angle in deg as a float32 kept in [0, 360)
float toFloatDegrees(double degrees);

/// Course over ground (degrees from true North, clockwise, in [0, 360)) of
/// an ENU velocity; 360, AIS's "not available", below kMinCourseSpeed.
double courseOf(double east_mps, double north_mps);

/// Horizontal speed below which a course is not defined, m/s.
constexpr double kMinCourseSpeed = 0.05;

/// Local ENU position of a WGS84 point: x and y projected from latitude and
/// longitude (degrees), z the altitude (metres from sea level).
gz::math::Vector3d geoToLocal(
    const gz::math::SphericalCoordinates& origin,
    double latitude_deg,
    double longitude_deg,
    double altitude_m);

/// The inverse of geoToLocal: (latitude deg, longitude deg, altitude m) of a local ENU position
gz::math::Vector3d localToGeo(
    const gz::math::SphericalCoordinates& origin,
    const gz::math::Vector3d& local);

}  // namespace lotusim::common::geo

#endif
