/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_common/transforms.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components/SphericalCoordinates.hh>

namespace lotusim::common {

namespace {

using gz::math::SphericalCoordinates;

// The plane the world is projected on: tangent at the origin's latitude and
// longitude, x East. <elevation> and <heading_deg> are left out (see
// transforms.hpp).
SphericalCoordinates tangentPlaneAt(const SphericalCoordinates& world_origin)
{
    return SphericalCoordinates(
        world_origin.Surface(),
        world_origin.LatitudeReference(),
        world_origin.LongitudeReference(),
        0.0,
        gz::math::Angle(0.0));
}

}  // namespace

//============================================================================
// Angles
//============================================================================

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
    return degToRad(90.0 - heading_deg);
}

double yawToHeading(double yaw_rad)
{
    return wrapDegrees(90.0 - radToDeg(yaw_rad));
}

double courseOf(double east_mps, double north_mps)
{
    if (std::hypot(east_mps, north_mps) < kMinCourseSpeed) {
        return kCourseNotAvailable;
    }
    return wrapDegrees(radToDeg(std::atan2(east_mps, north_mps)));
}

float toFloatDegrees(double degrees)
{
    const float f = static_cast<float>(wrapDegrees(degrees));
    return f >= 360.0f ? 0.0f : f;
}

//============================================================================
// Frames
//============================================================================

gz::math::Quaterniond quatChangeFrame(
    const gz::math::Quaterniond& q,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC)
{
    const gz::math::Matrix3d R(q);
    // const gz::math::Matrix3d R_new = worldC.Inverse() * R * bodyC;  // worldC is self-inverse, so no need to call Inverse()
    const gz::math::Matrix3d R_new = worldC * R * bodyC;
    gz::math::Quaterniond q_new(R_new);
    q_new.Normalize();
    return q_new;
}

gz::math::Pose3d poseChangeFrame(
    const gz::math::Pose3d& pose,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC)
{
    return gz::math::Pose3d(
        // worldC.Inverse() * pose.Pos(), // worldC is self-inverse, so no need to call Inverse()
        worldC * pose.Pos(),
        quatChangeFrame(pose.Rot(), worldC, bodyC));
}

gz::math::Vector3d vecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    // return bodyC.Inverse() * v;  // bodyC is self-inverse, so no need to call Inverse()
    return bodyC * v;
}

gz::math::Vector3d pseudoVecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    // return bodyC.Determinant() * (bodyC.Inverse() * v);
    return bodyC.Determinant() * (bodyC * v);
}

gz::math::Vector3d worldVelToTargetBody(
    const gz::math::Vector3d& v_world,
    const gz::math::Quaterniond& q_world_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    const gz::math::Vector3d v_body_source =
        q_world_attitude.RotateVectorReverse(v_world);
    return isPseudoVector ? pseudoVecBodyChangeFrame(v_body_source, bodyC)
                          : vecBodyChangeFrame(v_body_source, bodyC);
}

gz::math::Vector3d targetBodyVelToWorld(
    const gz::math::Vector3d& v_body_target,
    const gz::math::Quaterniond& q_world_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    const gz::math::Vector3d v_body_source =
        isPseudoVector ? pseudoVecBodyChangeFrame(v_body_target, bodyC)
                       : vecBodyChangeFrame(v_body_target, bodyC);
    return q_world_attitude.RotateVector(v_body_source);
}

//============================================================================
// Geographic positions
//============================================================================

gz::math::Vector3d geoToLocal(
    const SphericalCoordinates& origin,
    double latitude_deg,
    double longitude_deg,
    double altitude_m)
{
    // The point at sea level, projected onto the tangent plane.
    const auto on_plane = tangentPlaneAt(origin).PositionTransform(
        {degToRad(latitude_deg), degToRad(longitude_deg), 0.0},
        SphericalCoordinates::SPHERICAL,
        SphericalCoordinates::LOCAL2);
    return {on_plane.X(), on_plane.Y(), altitude_m};
}

gz::math::Vector3d localToGeo(
    const SphericalCoordinates& origin,
    const gz::math::Vector3d& local)
{
    const SphericalCoordinates plane = tangentPlaneAt(origin);
    // The sea-level point whose projection is (x, y): on the origin's
    // vertical through (x, y), where the elevation is 0. The elevation grows
    // with z at a rate close to 1, so each step removes almost all of the
    // error; a few steps reach a micrometre at 1000 km.
    double z = 0.0;
    gz::math::Vector3d spherical;
    for (int i = 0; i < 8; ++i) {
        spherical = plane.PositionTransform(
            {local.X(), local.Y(), z},
            SphericalCoordinates::LOCAL2,
            SphericalCoordinates::SPHERICAL);
        if (std::abs(spherical.Z()) < 1e-6) {
            break;
        }
        z -= spherical.Z();
    }
    return {radToDeg(spherical.X()), radToDeg(spherical.Y()), local.Z()};
}

gz::math::Pose3d geoToPose(
    const SphericalCoordinates& origin,
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    double heading_deg)
{
    const auto xyz =
        geoToLocal(origin, latitude_deg, longitude_deg, altitude_m);
    return gz::math::Pose3d(
        xyz.X(), xyz.Y(), xyz.Z(), 0.0, 0.0, headingToYaw(heading_deg));
}

//============================================================================
// ECM wrappers
//============================================================================

std::optional<SphericalCoordinates> worldOrigin(
    const gz::sim::EntityComponentManager& _ecm)
{
    const auto* sphComp =
        _ecm.Component<gz::sim::components::SphericalCoordinates>(
            gz::sim::worldEntity(_ecm));
    if (sphComp == nullptr) {
        return std::nullopt;
    }
    return sphComp->Data();
}

std::optional<std::tuple<double, double, double>> LatLongToXY(
    const gz::sim::EntityComponentManager& _ecm,
    double lat,
    double lon,
    double alt)
{
    const auto origin = worldOrigin(_ecm);
    if (!origin) {
        return std::nullopt;
    }
    const auto xyz = geoToLocal(*origin, lat, lon, alt);
    return std::make_tuple(xyz.X(), xyz.Y(), xyz.Z());
}

std::optional<std::tuple<double, double, double>> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    double x,
    double y,
    double z)
{
    return XYToLatLong(_ecm, gz::sim::worldEntity(_ecm), x, y, z);
}

std::optional<std::tuple<double, double, double>> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::sim::Entity& worldEntity,
    double x,
    double y,
    double z)
{
    const auto* sphComp =
        _ecm.Component<gz::sim::components::SphericalCoordinates>(
            worldEntity);
    if (sphComp == nullptr) {
        return std::nullopt;
    }
    const auto geo = localToGeo(sphComp->Data(), {x, y, z});
    return std::make_tuple(geo.X(), geo.Y(), geo.Z());
}

}  // namespace lotusim::common
