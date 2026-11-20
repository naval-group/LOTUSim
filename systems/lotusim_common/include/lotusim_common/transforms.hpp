/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COMMON_TRANSFORMS_HPP
#define LOTUSIM_COMMON_TRANSFORMS_HPP

#include <cmath>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <optional>

#include "lotusim_common/geo_types.hpp"

/**
 * @brief Angle, frame and position conversions.
 *
 * The pure functions take what they convert (and the world origin) as
 * arguments and need no ECM; the ECM wrappers at the end look the world's
 * origin up and call them.
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
 * latitude and longitude onto the plane tangent at the latitude and
 * longitude of the world's <spherical_coordinates>, and z is the altitude.
 * Both directions use the same projection, so a position read back is the
 * position given. Its local scale error is about d^2 / 2R^2 at a distance d
 * from the origin (1e-4 at 100 km). Gazebo's own conversion instead returns
 * the 3D point on the tangent plane, whose z drops with the Earth's
 * curvature (about 785 m at 100 km): a vessel given at the surface ended up
 * far below it.
 *
 * The world's x axis points East: <heading_deg> is not supported and
 * <elevation> does not move anything, as z is the altitude from sea level.
 * Both are left out of the projection.
 */
namespace lotusim::common {

//============================================================================
// Angles
//============================================================================

/// Radians to degrees.
inline double radToDeg(double rad)
{
    return rad * (180.0 / M_PI);
}

/// Degrees to radians.
inline double degToRad(double deg)
{
    return deg * (M_PI / 180.0);
}

/// An angle in degrees brought into [0, 360).
double wrapDegrees(double degrees);

/// Heading (degrees from true North, clockwise) to ENU yaw (radians from
/// East, counter-clockwise).
double headingToYaw(double heading_deg);

/// ENU yaw (radians from East, counter-clockwise) to heading (degrees from
/// true North, clockwise, in [0, 360)).
double yawToHeading(double yaw_rad);

/// courseOf()'s value when the course is not defined (AIS's "not available").
constexpr double kCourseNotAvailable = 360.0;

/// Horizontal speed below which a course is not defined, m/s.
constexpr double kMinCourseSpeed = 0.05;

/// Course over ground (degrees from true North, clockwise, in [0, 360)) of
/// an ENU velocity; kCourseNotAvailable below kMinCourseSpeed.
double courseOf(double east_mps, double north_mps);

/// An angle in degrees as a float32 kept in [0, 360): a double just under
/// 360 would round to 360.0f.
float toFloatDegrees(double degrees);

//============================================================================
// Frames
//============================================================================

// World: ENU (East,North,Up) <-> NED (North,East,Down): x_ned=y_enu,
// y_ned=x_enu, z_ned=-z_enu
static const gz::math::Matrix3d kWorldEnuNed(0, 1, 0, 1, 0, 0, 0, 0, -1);
static const gz::math::Matrix3d kWorldNedEnu(0, 1, 0, 1, 0, 0, 0, 0, -1);
// Body: FLU (Fwd,Left,Up) <-> FRD (Fwd,Right,Down): u_frd=u_flu, v_frd=-v_flu,
// w_frd=-w_flu
static const gz::math::Matrix3d kBodyFluFrd(1, 0, 0, 0, -1, 0, 0, 0, -1);
static const gz::math::Matrix3d kBodyFrdFlu(1, 0, 0, 0, -1, 0, 0, 0, -1);

// World: ENU (East,North,Up) <-> EUN (East,Up,North): swap Y,Z (Unity)
static const gz::math::Matrix3d kWorldEnuEun(1, 0, 0, 0, 0, 1, 0, 1, 0);
// Body: FLU (Fwd,Left,Up) <-> FUL (Fwd,Up,Left): swap Y,Z -- mirrors the
// world swap above for this convention.
static const gz::math::Matrix3d kBodyFluFul(1, 0, 0, 0, 0, 1, 0, 1, 0);

// World: ENU (East,North,Up) <-> NEU (North,East,Up): swap X,Y (Unreal)
static const gz::math::Matrix3d kWorldEnuNeu(0, 1, 0, 1, 0, 0, 0, 0, 1);
// Body: FLU (Fwd,Left,Up) <-> FRU (Fwd,Right,Up): Left<->Right flip only.
static const gz::math::Matrix3d kBodyFluFru(1, 0, 0, 0, -1, 0, 0, 0, 1);

/**
 * @brief Change an attitude quaternion between world and body axis frames.
 *
 * @param q Source attitude quaternion.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The attitude quaternion in the target convention.
 */
gz::math::Quaterniond quatChangeFrame(
    const gz::math::Quaterniond& q,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC);

/**
 * @brief Change the position and attitude of a pose between conventions.
 *
 * @param pose Source pose.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The pose in the target convention.
 */
gz::math::Pose3d poseChangeFrame(
    const gz::math::Pose3d& pose,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC);

/**
 * @brief Relabel an ordinary vector from source to the target body frame.
 *
 * @note Ordinary body-frame vector (e.g. linear velocity): just the
 * relabeling, no extra sign.
 * @param v Vector in the body frame.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The vector in the target body frame.
 */
gz::math::Vector3d vecBodyChangeFrame(
    const gz::math::Vector3d& v,
    const gz::math::Matrix3d& bodyC);

/**
 * @brief Relabel a body-frame pseudovector, including handedness correction.
 *
 * @note Body-frame pseudovector (e.g. angular velocity): picks up an extra
 * sign of det(bodyC) relative to an ordinary vector whenever bodyC flips
 * handedness.
 * @param v Vector in the body frame.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The pseudovector in the target body frame.
 */
gz::math::Vector3d pseudoVecBodyChangeFrame(
    const gz::math::Vector3d& v,
    const gz::math::Matrix3d& bodyC);

/**
 * @brief Convert a world-frame velocity to the target body frame.
 *
 * @param v_world Velocity in the source world frame.
 * @param q_world_attitude Vehicle attitude, rotating from the source body
 * frame to the source world frame.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the target body frame.
 */
gz::math::Vector3d worldVelToTargetBody(
    const gz::math::Vector3d& v_world,
    const gz::math::Quaterniond& q_world_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector);

/**
 * @brief Convert a target body-frame velocity to the source world frame.
 *
 * @param v_body_target Velocity in the target body frame.
 * @param q_world_attitude Vehicle attitude, rotating from the source body
 * frame to the source world frame.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the source world frame.
 */
gz::math::Vector3d targetBodyVelToWorld(
    const gz::math::Vector3d& v_body_target,
    const gz::math::Quaterniond& q_world_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector);

//============================================================================
// Geographic positions
//============================================================================

/// Local ENU position of a WGS84 point: x and y projected from latitude and
/// longitude, z the altitude.
gz::math::Vector3d geoToLocal(
    const gz::math::SphericalCoordinates& origin,
    const GeoPoint& geo);

/// The inverse of geoToLocal: the WGS84 point of a local ENU position.
GeoPoint localToGeo(
    const gz::math::SphericalCoordinates& origin,
    const gz::math::Vector3d& local);

/// Local ENU pose of a vessel at a WGS84 point with a heading (degrees from
/// true North, clockwise), level: no roll and no pitch.
gz::math::Pose3d geoToPose(
    const gz::math::SphericalCoordinates& origin,
    const GeoPoint& geo,
    double heading_deg);

/// The geographic pose of a local ENU pose: the position through localToGeo,
/// the attitude unchanged (it is already in the ENU world frame).
GeoPose localToGeoPose(
    const gz::math::SphericalCoordinates& origin,
    const gz::math::Pose3d& local);

//============================================================================
// ECM wrappers: the same conversions with the world's origin
//============================================================================

/**
 * @brief The world's <spherical_coordinates>, or std::nullopt if it has
 * none.
 */
std::optional<gz::math::SphericalCoordinates> worldOrigin(
    const gz::sim::EntityComponentManager& _ecm);

/**
 * @brief Geographic pose of an entity: localToGeoPose of its world pose,
 * with the world's origin.
 *
 * @param entity Entity whose world pose is converted
 * @param _ecm
 * @return std::optional<GeoPose> std::nullopt if the world has no spherical
 * coordinates
 */
std::optional<GeoPose> worldGeoPose(
    const gz::sim::Entity& entity,
    const gz::sim::EntityComponentManager& _ecm);

/**
 * @brief geoToLocal with the world's origin.
 *
 * @param _ecm
 * @param geo WGS84 point
 * @return std::optional<gz::math::Vector3d> local ENU position, or
 * std::nullopt if the world has no spherical coordinates
 */
std::optional<gz::math::Vector3d> LatLongToXY(
    const gz::sim::EntityComponentManager& _ecm,
    const GeoPoint& geo);

/**
 * @brief localToGeo with the world's origin.
 *
 * @param _ecm Entity Component Manager used to retrieve spherical coordinates
 * @param local Local ENU position in metres
 * @return std::optional<GeoPoint> WGS84 point, or std::nullopt if the world
 * has no spherical coordinates
 */
std::optional<GeoPoint> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::math::Vector3d& local);

/**
 * @brief localToGeo with the origin of a world entity the caller already
 * holds.
 *
 * @param _ecm Entity Component Manager used to retrieve spherical coordinates
 * @param worldEntity World entity holding the spherical coordinates
 * @param local Local ENU position in metres
 * @return std::optional<GeoPoint> WGS84 point, or std::nullopt if the world
 * has no spherical coordinates
 */
std::optional<GeoPoint> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::sim::Entity& worldEntity,
    const gz::math::Vector3d& local);

}  // namespace lotusim::common

#endif
