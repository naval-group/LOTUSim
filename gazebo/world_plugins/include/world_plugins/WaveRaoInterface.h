#ifndef __WAVE_RAO_INTERFACE_HH__
#define __WAVE_RAO_INTERFACE_HH__

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <tuple>

namespace liquidai {
namespace gazebo {

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
 * @brief Base class for the different physics server interface. It is
 * reponsible for interfacing with the calculator of your choice. It is also
 * responsible for subscribing to the thruster commands your calculator
 * take.
 *
 */
class WaveRaoInterface {
public:
    WaveRaoInterface(){};

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

protected:
    gz::transport::Node m_gz_node;
};

// typedef std::shared_ptr<WaveRaoInterface> ClientPtr;

}  // namespace gazebo
}  // namespace liquidai
#endif