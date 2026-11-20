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
    const GeoPoint& geo)
{
    // The point at sea level, projected onto the tangent plane.
    const auto on_plane = tangentPlaneAt(origin).PositionTransform(
        {degToRad(geo.latitude), degToRad(geo.longitude), 0.0},
        SphericalCoordinates::SPHERICAL,
        SphericalCoordinates::LOCAL2);
    return {on_plane.X(), on_plane.Y(), geo.altitude};
}

GeoPoint localToGeo(
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
    const GeoPoint& geo,
    double heading_deg)
{
    const auto xyz = geoToLocal(origin, geo);
    return gz::math::Pose3d(
        xyz.X(), xyz.Y(), xyz.Z(), 0.0, 0.0, headingToYaw(heading_deg));
}

GeoPose localToGeoPose(
    const SphericalCoordinates& origin,
    const gz::math::Pose3d& local)
{
    return {localToGeo(origin, local.Pos()), local.Rot()};
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

std::optional<GeoPose> worldGeoPose(
    const gz::sim::Entity& entity,
    const gz::sim::EntityComponentManager& _ecm)
{
    const auto origin = worldOrigin(_ecm);
    if (!origin) {
        return std::nullopt;
    }
    return localToGeoPose(*origin, gz::sim::worldPose(entity, _ecm));
}

std::optional<gz::math::Vector3d> LatLongToXY(
    const gz::sim::EntityComponentManager& _ecm,
    const GeoPoint& geo)
{
    const auto origin = worldOrigin(_ecm);
    if (!origin) {
        return std::nullopt;
    }
    return geoToLocal(*origin, geo);
}

std::optional<GeoPoint> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::math::Vector3d& local)
{
    return XYToLatLong(_ecm, gz::sim::worldEntity(_ecm), local);
}

std::optional<GeoPoint> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::sim::Entity& worldEntity,
    const gz::math::Vector3d& local)
{
    const auto* sphComp =
        _ecm.Component<gz::sim::components::SphericalCoordinates>(
            worldEntity);
    if (sphComp == nullptr) {
        return std::nullopt;
    }
    return localToGeo(sphComp->Data(), local);
}

}  // namespace lotusim::common
