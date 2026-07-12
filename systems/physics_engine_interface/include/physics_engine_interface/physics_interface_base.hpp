/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_PHYSICS_ENGINE_INTERFACE_BASE_HH_
#define LOTUSIM_PHYSICS_ENGINE_INTERFACE_BASE_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>

#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

enum class InterfaceType
{
    XDynWebSocket,
    ROS2Interface,
    Unknown
};

static std::unordered_map<std::string, InterfaceType> InterfaceTypeMap{
    {"XDYNWEBSOCKET", InterfaceType::XDynWebSocket},
    {"ROS2", InterfaceType::ROS2Interface},
    {"UNKNOWN", InterfaceType::Unknown}};

enum class DomainType
{
    Aerial,
    Surface,
    Underwater,
    Unknown
};

static std::unordered_map<std::string, DomainType> DomainTypeMap{
    {"AERIAL", DomainType::Aerial},
    {"SURFACE", DomainType::Surface},
    {"UNDERWATER", DomainType::Underwater},
    {"UNKNOWN", DomainType::Unknown}};

static std::unordered_map<DomainType, std::string> DomainTypeToStringMap{
    {DomainType::Aerial, "Aerial"},
    {DomainType::Surface, "Surface"},
    {DomainType::Underwater, "Underwater"},
    {DomainType::Unknown, "Unknown"}};

/**
 * @brief Struct to hold vessel information in ENU frame
 *
 */
struct VesselInformation {
    double time;
    gz::sim::Entity entity;
    gz::math::Pose3d pose;
    gz::math::Vector3d lin_vel;
    gz::math::Vector3d ang_vel;
};

/**
 * @brief Base class for the different physics server interface.
 * It is reponsible for interfacing with the physics engine of your choice. It
 * is also responsible for subscribing to the thruster commands your engine
 * takes
 * The model must have a link named base_link
 */
class PhysicsInterfaceBase {
public:
    PhysicsInterfaceBase(const std::string& interface_name = "")
    {
        std::string logger_name = interface_name + "_physics_engine_output";
        m_engine_logger = logger::createConsoleAndFileLogger(
            logger_name,
            logger_name + ".txt");
    };

    virtual ~PhysicsInterfaceBase() = default;

    /**
     * @brief Factory method create a Derived Interface class object
     *
     * @param entity
     * @param model_name
     * @param protocol_type
     * @param sdf
     * @param cmd
     * @param logger
     * @param domain_type
     * @return std::shared_ptr<PhysicsInterfaceBase>
     */
    static std::shared_ptr<PhysicsInterfaceBase> createInterface(
        const InterfaceType& protocol_type,
        std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>> cmd,
        std::shared_ptr<spdlog::logger> logger);

    /**
     * @brief Configure Interface upon creation
     *
     * @param entity
     * @param model_name
     * @param protocol_type
     * @param sdf
     * @param domain_type
     * @return true
     * @return false
     */
    virtual bool configureInterface(
        const gz::sim::Entity& entity,
        const std::string& model_name,
        const sdf::ElementPtr sdf,
        const DomainType& domain_type = DomainType::Unknown) = 0;

    /**
     * @brief Model will be deleted and interface to be deactivated and
     * destroyed
     *
     * @param _entity
     * @param domain_type Optional
     * @return true
     * @return false
     */
    virtual bool removeInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown)
    {
        return true;
    };

    /**
     * @brief Activate the Interface to be used
     *
     * @param _entity
     * @param domain_type Optional
     * @return true
     * @return false
     */
    virtual bool activateInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) = 0;

    /**
     * @brief Close the Interface when vessel transit
     * Interface must deactivate. Failure to deactivate must be handled on
     * interface level
     *
     * Interface must be able to be activated again in the future and not be
     * destroyed. Freeing of interface object is done by removeInterface or
     * garbage collector
     *
     * @param _entity
     * @param domain_type Optional
     * @return true
     * @return false
     */
    virtual bool deactivateInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) = 0;

    /**
     * @brief Get the New State object
     *
     * @param _entity
     * @param previous_state
     * @param time_dif
     * @return std::optional<std::tuple<VesselInformation, DomainType>> New
     * state update of model and the next domain
     */
    virtual std::optional<std::tuple<VesselInformation, DomainType>>
    getNewState(
        const gz::sim::Entity& _entity,
        const VesselInformation& previous_state,
        float time_dif) = 0;

    /**
     * @brief Get the URI of the Interface
     *
     * @param _entity
     * @param domain_type Optional
     * @return std::string
     */
    virtual std::string getURI(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) = 0;

    /**
     * @brief Set the Logger object
     *
     * @param logger
     */
    void setLogger(std::shared_ptr<spdlog::logger> logger)
    {
        m_logger = logger;
    }

    void logEngineState(
        const VesselInformation& state,
        const DomainType& domain,
        const std::string& model_name = "")
    {
        if (m_engine_logger) {
            std::string excelRow = fmt::format(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                DomainTypeToStringMap[domain],
                state.time,
                state.entity,
                model_name,
                state.pose.X(),
                state.pose.Y(),
                state.pose.Z(),
                state.pose.Rot().X(),
                state.pose.Rot().Y(),
                state.pose.Rot().Z(),
                state.pose.Rot().W(),
                state.lin_vel.X(),
                state.lin_vel.Y(),
                state.lin_vel.Z(),
                state.ang_vel.X(),
                state.ang_vel.Y(),
                state.ang_vel.Z());
            m_engine_logger->debug(excelRow);
        }
    }

    void setSharedCmd(
        std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>> _cmd)
    {
        m_models_cmd_map_ptr = _cmd;
    }

protected:
    /**
     * @brief Runtime logger of the physics plugin
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief Ptr to hold cmd. Passed from main plugin
     * Mapping of vessel entity to cmd string
     *
     */
    std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>>
        m_models_cmd_map_ptr;

    /**
     * @brief Physics engine logger in excel format to log the 9 D.O.F
     * of the object
     *
     */
    std::shared_ptr<spdlog::logger> m_engine_logger;
};

// typedef std::shared_ptr<PhysicsInterfaceBase> ClientPtr;

}  // namespace lotusim::gazebo
#endif