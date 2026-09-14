/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_VESSEL_INFORMATION_HH_
#define LOTUSIM_VESSEL_INFORMATION_HH_

/**
 * @brief Defines the vessel state representation and coordinate-convention conversion utilities.
 *
 * This file provides a unified data structure for representing a vessel's
 * pose and motion in a simulation environment, along with logic to convert
 * that state between different coordinate conventions used by different
 * robotics and game-engine frameworks.
 *
 * The central type, VesselInformation, stores:
 * - the active coordinate convention
 * - simulation time
 * - the associated Gazebo entity
 * - position and orientation as a pose
 * - linear velocity
 * - angular velocity
 *
 * Supported conventions include Gazebo/ENU_FLU, NED_FRD, EUN_FUL, and
 * NEU_FRU. The class exposes conversion methods for transforming states
 * from the internal Gazebo convention into external formats such as xDyn,
 * Unity, and Unreal, as well as a factory method for converting xDyn data
 * back into the Gazebo representation.
 *
 * The implementation handles the subtle differences between world-frame and
 * body-frame velocity representations, as well as quaternion and axis
 * re-labeling needed to preserve correctness across conventions.
 *
 * @note Velocities are stored in the world frame for the GAZEBO/ENU_FLU
 * convention, while other conventions store velocities in the body frame.
 */

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/System.hh>

#include <ostream>

namespace lotusim::gazebo {

/**
 * @brief Coordinate conventions supported by VesselInformation.
 */
enum class Convention : int {
    UNKNOWN = 0,
    GAZEBO, //< Gazebo convention: ENU world and FLU body axes, with world-frame velocities.
    ENU_FLU, //< East-North-Up world and Forward-Left-Up body axes, with body-frame velocities.
    NED_FRD, //< North-East-Down world and Forward-Right-Down body axes, with body-frame velocities.
    EUN_FUL, //< Unity convention.
    NEU_FRU, //< Unreal convention.
    ERROR
};

/**
 * @brief Vessel state expressed in a supported coordinate convention.
 *
 * In the GAZEBO/ENU_FLU convention, velocities are stored in the world
 * frame. In all other conventions, velocities are stored in the body frame.
 */
struct VesselInformation {
    Convention convention; ///< Coordinate convention of this state.
    double time; ///< Simulation time associated with this state.
    gz::sim::Entity entity; ///< Gazebo entity associated with this vessel.
    gz::math::Pose3d pose; ///< Position and attitude.
    gz::math::Vector3d lin_vel; ///< Linear velocity in the applicable frame.
    gz::math::Vector3d ang_vel; ///< Angular velocity in the applicable frame.

    /** @brief Construct an empty state in the GAZEBO convention. */
    VesselInformation(): convention(Convention::GAZEBO), time(0.0), entity(), pose(), lin_vel(), ang_vel(){};

    /**
     * @brief Construct a vessel state with explicit values.
     * @param conv Coordinate convention.
     * @param t Simulation time.
     * @param p Position and attitude.
     * @param lv Linear velocity.
     * @param av Angular velocity.
     */
    VesselInformation(Convention conv, double t, const gz::math::Pose3d& p, const gz::math::Vector3d& lv, const gz::math::Vector3d& av):
        convention(conv), time(t), pose(p), lin_vel(lv), ang_vel(av) {};

    /** @brief Convert a GAZEBO state to the xdyn NED_FRD convention. */
    VesselInformation to_xdyn() const;

    /** @brief Convert a GAZEBO state to the Unity EUN_FUL convention. */
    VesselInformation to_unity() const;

    /** @brief Convert a GAZEBO state to the Unreal NEU_FRU convention. */
    VesselInformation to_unreal() const;

    /**
     * @brief Convert xdyn data from NED_FRD to the GAZEBO convention.
     * @param xyz Position in the NED world frame.
     * @param quaternion Attitude in the NED_FRD convention.
     * @param uvw Linear velocity in the FRD body frame.
     * @param pqr Angular velocity in the FRD body frame.
     * @return The converted vessel state in the GAZEBO convention.
     */
    static VesselInformation from_xdyn(
        const gz::math::Vector3d& xyz,
        const gz::math::Quaterniond& quaternion,
        const gz::math::Vector3d& uvw,
        const gz::math::Vector3d& pqr);

    /** @brief Inplace convert a GAZEBO state to the xdyn NED_FRD convention. */
    void convert_to_xdyn();

    /** @brief Inplace convert a GAZEBO state to the Unity EUN_FUL convention. */
    void convert_to_unity();

    /** @brief Inplace convert a GAZEBO state to the Unreal NEU_FRU convention. */
    void convert_to_unreal();

    /** @brief Convert xdyn data from NED_FRD to the GAZEBO convention.*/
    void convert_from_xdyn();
};

/**
 * @brief Stream a coordinate convention name.
 * @param os Output stream.
 * @param s Convention to print.
 * @return The output stream.
 */
std::ostream& operator<<(std::ostream& os, Convention s);

}

#endif