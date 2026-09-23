/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/vessel_information.hpp"

#include <lotusim_common/common.hpp>

#include <iostream>
#include <stdexcept>


namespace lotusim::gazebo {

namespace common = lotusim::common;

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

// quatChangeFrame, poseChangeFrame, vecBodyChangeFrame,
// pseudoVecBodyChangeFrame, worldVelToTargetBody and targetBodyVelToWorld
// live in lotusim_common/common.hpp: they take arbitrary axis conversion
// matrices and don't depend on Convention or VesselInformation, so any
// system doing frame relabeling can reuse them.
//
// Every call below passes q_enu_attitude / pose.Rot() as the world-attitude
// argument: this->lin_vel / this->ang_vel (ENU_FLU convention) are stored in
// the ENU ground (WORLD) frame, while every other convention in this file
// stores body-frame velocities. Converting world -> body-of-target therefore
// needs an extra step that pure axis relabeling doesn't: worldVelToTargetBody
// first undoes the vehicle's own attitude rotation to get the velocity in FLU
// body-frame components, THEN relabels those FLU components into the
// target's body axes (and targetBodyVelToWorld does the inverse).

/** @brief Convert this GAZEBO state to xdyn's NED_FRD convention. */
VesselInformation VesselInformation::to_xdyn() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::NED_FRD;
    v.time = time;
    v.entity = entity;
    v.pose = common::poseChangeFrame(pose, common::kWorldEnuNed, common::kBodyFluFrd);
    v.lin_vel = common::worldVelToTargetBody(lin_vel, pose.Rot(), common::kBodyFluFrd, /*isPseudoVector=*/false);
    v.ang_vel = common::worldVelToTargetBody(ang_vel, pose.Rot(), common::kBodyFluFrd, /*isPseudoVector=*/true);

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
    v.pose = common::poseChangeFrame(pose, common::kWorldEnuEun, common::kBodyFluFul);
    v.lin_vel = common::worldVelToTargetBody(lin_vel, pose.Rot(), common::kBodyFluFul, /*isPseudoVector=*/false);
    v.ang_vel = common::worldVelToTargetBody(ang_vel, pose.Rot(), common::kBodyFluFul, /*isPseudoVector=*/true);
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
    v.pose = common::poseChangeFrame(pose, common::kWorldEnuNeu, common::kBodyFluFru);
    v.lin_vel = common::worldVelToTargetBody(lin_vel, pose.Rot(), common::kBodyFluFru, /*isPseudoVector=*/false);
    v.ang_vel = common::worldVelToTargetBody(ang_vel, pose.Rot(), common::kBodyFluFru, /*isPseudoVector=*/true);
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
    s.pose = common::poseChangeFrame(gz::math::Pose3d(ned_xyz, ned_quaternion), common::kWorldEnuNed, common::kBodyFluFrd);
    s.lin_vel = common::targetBodyVelToWorld(ned_uvw, s.pose.Rot(), common::kBodyFluFrd, /*isPseudoVector=*/false);
    s.ang_vel = common::targetBodyVelToWorld(ned_pqr, s.pose.Rot(), common::kBodyFluFrd, /*isPseudoVector=*/true);

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
