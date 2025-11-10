/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_RENDER_INTERFACE_HPP_
#define LOTUSIM_RENDER_INTERFACE_HPP_

#include <chrono>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <memory>
#include <sdf/sdf.hh>
#include <string>
#include <utility>
#include <vector>

#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

/**
 * @brief Base class for Render Interface.
 * All different type of render interface must be able to handle all these
 * cases.
 *
 */
class RenderInterfaceBase {
public:
    RenderInterfaceBase(
        const std::string& world_name,
        std::shared_ptr<spdlog::logger> logger)
        : m_logger(logger), m_world_name(world_name) {};

    virtual ~RenderInterfaceBase() = default;

    /**
     * @brief Configure the interface with SDF parameters
     *
     * @param _sdf SDF element containing configuration
     * @return true if configuration successful
     * @return false otherwise
     */
    virtual bool configureInterface(
        const std::shared_ptr<const sdf::Element>& _sdf) = 0;

    /**
     * @brief Send the simulation frame update to renderer
     *
     * @param runTime Current simulation time
     * @param poses Vector of vessel names and their poses
     * @return true if send successful
     * @return false otherwise
     */
    virtual bool sendPosition(
        const std::chrono::steady_clock::duration& runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>>& poses) = 0;

    /**
     * @brief Create a Vessel in the simulation.
     * lotus param will be given and it is up to the user to choose how they
     * will like to interface
     *
     * @param vessel_name
     * @param pose
     * @param sdfptr lotus_param sdfptr
     * @return true
     * @return false
     */
    virtual bool createVessel(
        const std::string& vessel_name,
        const gz::math::Pose3d& pose,
        sdf::ElementPtr sdfptr) = 0;

    /**
     * @brief Destroy vessel in the simulation
     *
     * @param vessel_name
     * @return true
     * @return false
     */
    virtual bool destroyVessel(const std::string& vessel_name) = 0;

    /**
     * @brief CustomPreupdate loop if required
     *
     * @param _info
     * @param _ecm
     * @return true
     * @return false
     */
    virtual bool customPreUpdates(
        const gz::sim::UpdateInfo&,
        gz::sim::EntityComponentManager&)
    {
        return true;
    };

    /**
     * @brief CustomUpdate loop if required
     *
     * @param _info
     * @param _ecm
     * @return true
     * @return false
     */
    virtual bool customUpdates(
        const gz::sim::UpdateInfo&,
        const gz::sim::EntityComponentManager&)
    {
        return true;
    };

protected:
    std::shared_ptr<spdlog::logger> m_logger;
    std::string m_world_name;
};

}  // namespace lotusim::gazebo

#endif