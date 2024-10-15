#ifndef __PHYSICS_ENGINE_INTERFACE_BASE_HH__
#define __PHYSICS_ENGINE_INTERFACE_BASE_HH__

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <tuple>

#include "logging_system/logger.hpp"

namespace lotusim::gazebo {

using json = nlohmann::json;

enum class ConnectionType
{
    XDynWebSocket,
    XDynGRPC,
    Manual,
    Unknown
};
static std::unordered_map<std::string, ConnectionType> ConnectionTypeMap{
    {"XDynWebSocket", ConnectionType::XDynWebSocket},
    {"XDynGRPC", ConnectionType::XDynGRPC},
    {"Manual", ConnectionType::Manual},
    {"Unknown", ConnectionType::Unknown}};

enum class DomainType
{
    Aerial,
    Surface,
    Underwater
};
static std::unordered_map<std::string, DomainType> DomainTypeMap{
    {"Aerial", DomainType::Aerial},
    {"Surface", DomainType::Surface},
    {"Underwater", DomainType::Underwater}};

/**
 * @brief Base class for the different physics server interface.
 * It is reponsible for interfacing with the physics engine of your choice. It
 * is also responsible for subscribing to the thruster commands your engine
 * takes
 *
 */
class PhysicsInterfaceBase {
public:
    PhysicsInterfaceBase()
    {
        m_engine_logger = logger::createConsoleAndFileLogger(
            "physics_engine_output",
            "physics_engine_output.txt");
    };

    /**
     * @brief Get the New State object
     *
     * @param name
     * @param previous_state  Json will have time, position, orientation in
     * quaternion
     * @param time_dif In milliseconds
     * @return
     */
    virtual std::optional<std::tuple<json, DomainType>> getNewState(
        const gz::sim::Entity &_entity,
        const json &previous_state,
        float time_dif) = 0;

    /**
     * @brief Create a Connection for the interface
     *
     * @param name
     * @param thrusters_name
     * @param uri
     * @return true
     * @return false
     */
    virtual bool createConnection(
        const gz::sim::Entity &_entity,
        const std::string &_name,
        const sdf::ElementPtr _sdf) = 0;

    /**
     * @brief Activate the connection to be used
     *
     * @param _entity
     * @return true
     * @return false
     */
    virtual bool activateConnection(const gz::sim::Entity &_entity) = 0;

    /**
     * @brief Close the connection when vessel transit
     *
     * @param _entity
     * @return true
     * @return false
     */
    virtual bool deactivateConnection(const gz::sim::Entity &_entity) = 0;

    /**
     * @brief Get the URI of the connection
     *
     * @param _entity
     * @return std::string
     */
    virtual std::string getURI(const gz::sim::Entity &_entity) = 0;

    /**
     * @brief Set the Logger object
     *
     * @param logger
     */
    void setLogger(std::shared_ptr<spdlog::logger> logger)
    {
        m_logger = logger;
    }

protected:
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief Physics engine logger in excel format
     *
     */
    std::shared_ptr<spdlog::logger> m_engine_logger;

    gz::transport::Node m_gz_node;
};

// typedef std::shared_ptr<PhysicsInterfaceBase> ClientPtr;

}  // namespace lotusim::gazebo
#endif