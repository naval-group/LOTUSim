/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/vessel_information.hpp"

#include <iostream>
#include <stdexcept>


namespace lotusim::gazebo {

/**
 * @brief Stream a human-readable coordinate convention name.
 */
std::ostream& operator<<(std::ostream& os, Convention s)
{
    switch (s) {
        case Convention::UNKNOWN:  return os << "Unknown";
        case Convention::GAZEBO:   return os << "GAZEBO";
        case Convention::ENU_FLU:  return os << "ENU_FLU";
        case Convention::NED_FRD:  return os << "NED_FRD";
        case Convention::EUN_FUL:  return os << "EUN_FUL";
        case Convention::NEU_FRU:  return os << "NEU_FRU";
    }
    return os << "Unknown";
}

// World: ENU (East,North,Up) <-> NED (North,East,Down): x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu
static const gz::math::Matrix3d kWorldEnuNed(0, 1, 0,  1, 0, 0,  0, 0, -1);
static const gz::math::Matrix3d kWorldNedEnu(0, 1, 0,  1, 0, 0,  0, 0, -1);
// Body: FLU (Fwd,Left,Up) <-> FRD (Fwd,Right,Down): u_frd=u_flu, v_frd=-v_flu, w_frd=-w_flu
static const gz::math::Matrix3d kBodyFluFrd(1, 0, 0,  0, -1, 0,  0, 0, -1);
static const gz::math::Matrix3d kBodyFrdFlu(1, 0, 0,  0, -1, 0,  0, 0, -1);

// World: ENU (East,North,Up) <-> EUN (East,Up,North): swap Y,Z (Unity)
static const gz::math::Matrix3d kWorldEnuEun(1, 0, 0,  0, 0, 1,  0, 1, 0);
// Body: FLU (Fwd,Left,Up) <-> FUL (Fwd,Up,Left): swap Y,Z -- mirrors the
// world swap above for this convention.
static const gz::math::Matrix3d kBodyFluFul(1, 0, 0,  0, 0, 1,  0, 1, 0);

// World: ENU (East,North,Up) <-> NEU (North,East,Up): swap X,Y (Unreal)
static const gz::math::Matrix3d kWorldEnuNeu(0, 1, 0,  1, 0, 0,  0, 0, 1);
// Body: FLU (Fwd,Left,Up) <-> FRU (Fwd,Right,Up): Left<->Right flip only.
static const gz::math::Matrix3d kBodyFluFru(1, 0, 0,  0, -1, 0,  0, 0, 1);

// ---------------------------------------------------------------------------
// Generic, always-correct conversions
// ---------------------------------------------------------------------------
/**
 * @brief Change an attitude quaternion between world and body axis frames.
 * @param q Source attitude quaternion.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The attitude quaternion in the target convention.
 */
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

/**
 * @brief Change the position and attitude of a pose between conventions.
 * @param pose Source pose.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The pose in the target convention.
 */
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

/**
 * @brief Relabel an ordinary vector from source to the target body frame.
 * @note Ordinary body-frame vector (e.g. linear velocity): just the relabeling,
 * no extra sign.
 * @param v Vector in the body frame.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The vector in the target body frame.
 */
inline gz::math::Vector3d vecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    // return bodyC.Inverse() * v;  // bodyC is self-inverse, so no need to call Inverse()
    return bodyC * v;
}

/**
 * @brief Relabel a body-frame pseudovector, including handedness correction.
 * @note Body-frame pseudovector (e.g. angular velocity): picks up an extra sign
 * of det(bodyC) relative to an ordinary vector whenever bodyC flips
 * handedness. For kBodyFluFrd (det=+1) this is a no-op; for kBodyFluFul and
 * kBodyFluFru (det=-1 each) it is not, which is why angular velocity and
 * linear velocity need separate helpers even though they look similar.
*/
 inline gz::math::Vector3d pseudoVecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    // return bodyC.Determinant() * (bodyC.Inverse() * v);
    return bodyC.Determinant() * (bodyC * v);
}

/**
 * @brief Convert a world-frame velocity to the target body frame.
 * @param v_world_enu Velocity in the ENU world frame.
 * @param q_enu_attitude Vehicle attitude in the ENU_FLU convention.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the target body frame.
 * @note this->lin_vel / this->ang_vel (ENU_FLU convention) are stored in the
 * ENU ground (WORLD) frame. Every other convention in this file stores
 * body-frame velocities. Converting world -> body-of-target therefore
 * needs an extra step that pure axis relabeling doesn't: first undo the
 * vehicle's own attitude rotation to get the velocity in FLU body-frame
 * components, THEN relabel those FLU components into the target's body
 * axes.
 */
gz::math::Vector3d worldVelToTargetBody(
    const gz::math::Vector3d& v_world_enu,
    const gz::math::Quaterniond& q_enu_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    const gz::math::Vector3d v_body_flu =
        q_enu_attitude.RotateVectorReverse(v_world_enu);
    return isPseudoVector ? pseudoVecBodyChangeFrame(v_body_flu, bodyC)
                          : vecBodyChangeFrame(v_body_flu, bodyC);
}

/**
 * @brief Convert a target body-frame velocity to the ENU world frame.
 * @param v_body_target Velocity in the target body frame.
 * @param q_enu_attitude Vehicle attitude in the ENU_FLU convention.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the ENU world frame.
 * @note Inverse of the above: a body-frame velocity in some target convention
 * (e.g. xdyn's FRD uvw/pqr) needs relabeling into FLU body-frame
 * components, then rotating by the (already-computed) ENU attitude to
 * land in ENU_FLU's world-frame velocity storage.
 */
gz::math::Vector3d targetBodyVelToWorld(
    const gz::math::Vector3d& v_body_target,
    const gz::math::Quaterniond& q_enu_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    const gz::math::Vector3d v_body_flu =
        isPseudoVector ? pseudoVecBodyChangeFrame(v_body_target, bodyC)
                       : vecBodyChangeFrame(v_body_target, bodyC);
    return q_enu_attitude.RotateVector(v_body_flu);
}

/** @brief Convert this GAZEBO state to xdyn's NED_FRD convention. */
VesselInformation VesselInformation::to_xdyn() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::NED_FRD;
    v.time = time;
    v.entity = entity;
    v.pose = poseChangeFrame(pose, kWorldEnuNed, kBodyFluFrd);
    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/true);

    return v;
}

/**
 * @brief Convert this GAZEBO state to Unity's EUN_FUL convention.
 */
VesselInformation VesselInformation::to_unity() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::EUN_FUL;
    v.time = time;
    v.entity = entity;
    v.pose = poseChangeFrame(pose, kWorldEnuEun, kBodyFluFul);
    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFul, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFul, /*isPseudoVector=*/true);
    return v;
}

/**
 * @brief Convert this GAZEBO state to Unreal's NEU_FRU convention.
 */
VesselInformation VesselInformation::to_unreal() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::NEU_FRU;
    v.time = time;
    v.entity = entity;
    v.pose = poseChangeFrame(pose, kWorldEnuNeu, kBodyFluFru);
    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFru, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFru, /*isPseudoVector=*/true);
    return v;
}

/**
 * @brief Convert xdyn NED_FRD data to a GAZEBO state.
 */
VesselInformation VesselInformation::from_xdyn(
        const gz::math::Vector3d& ned_xyz,
        const gz::math::Quaterniond& ned_quaternion,
        const gz::math::Vector3d& ned_uvw,   // body(FRD)-frame linear velocity
        const gz::math::Vector3d& ned_pqr)   // body(FRD)-frame angular velocity
{
    VesselInformation s;
    s.convention = Convention::GAZEBO;
    s.pose = poseChangeFrame(gz::math::Pose3d(ned_xyz, ned_quaternion), kWorldEnuNed, kBodyFluFrd);
    s.lin_vel = targetBodyVelToWorld(ned_uvw, s.pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/false);
    s.ang_vel = targetBodyVelToWorld(ned_pqr, s.pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/true);

    return s;
}

/** @brief Inplace convert a GAZEBO state to the xdyn NED_FRD convention. */
void VesselInformation::convert_to_xdyn()
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");
    gz::sim::Entity entity_bck = this->entity;
    *this = to_xdyn();
    this->entity = entity_bck;
}

/** @brief Inplace convert a GAZEBO state to the Unity EUN_FUL convention. */
void VesselInformation::convert_to_unity()
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");
    gz::sim::Entity entity_bck = this->entity;
    *this = to_unity();
    this->entity = entity_bck;
}

/** @brief Inplace convert a GAZEBO state to the Unreal NEU_FRU convention. */
void VesselInformation::convert_to_unreal()
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");
    gz::sim::Entity entity_bck = this->entity;
    *this = to_unreal();
    this->entity = entity_bck;
}

/** @brief Convert xdyn data from NED_FRD to the GAZEBO convention.*/
void VesselInformation::convert_from_xdyn()
{
    if (convention != Convention::NED_FRD)
        throw std::runtime_error("Invalid convention");
    gz::sim::Entity entity_bck = this->entity;
    *this = from_xdyn(pose.Pos(), pose.Rot(), lin_vel, ang_vel);
    this->entity = entity_bck;
}

}