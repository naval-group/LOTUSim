#ifndef __XDYN_WEBSOCKET_HH__
#define __XDYN_WEBSOCKET_HH__

#include <future>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "PhysicsInterfaceBase.h"

namespace lotusim::gazebo {

namespace Weblib = websocketpp::lib;
using hdl = websocketpp::connection_hdl;
using Client = websocketpp::client<websocketpp::config::asio_client>;
using json = nlohmann::json;

constexpr unsigned short DEFAULT_WEBSOCKET_TIMEOUT = 1000;

/**
 * @brief A Singleton class to connect to xdyn through websocket.
 *
 * A singleton class to reuse the client class
 * On sending req to xdyn and receiving msg, there is a frame conversion
 * Xdyn having a ned convention and right-hand axis convention
 *
 * m_vessels_cmd_map_ptr String is:
 * {
    "PSPropRudd(P/D)": 0.79,
    "PSPropRudd(beta)": 0.0,
    "PSPropRudd(rpm)": 0.0,
    "SBPropRudd(P/D)": 0.79,
    "SBPropRudd(beta)": 0.0,
    "SBPropRudd(rpm)": 0.0
    }
 *
 */
class XdynWebsocket : public PhysicsInterfaceBase {
public:
    XdynWebsocket();

    ~XdynWebsocket();

    void operator=(const XdynWebsocket &) = delete;

    XdynWebsocket(XdynWebsocket &other) = delete;

    static std::shared_ptr<XdynWebsocket> getInstance(
        const gz::sim::Entity &_entity,
        const std::string &_name);

    /**
     * @brief Get the New State object using given vessel state
     *
     * @param _entity
     * @param previous_state
     * @param time_dif
     * @return std::optional<json>
     */
    std::optional<std::tuple<json, DomainType>> getNewState(
        const gz::sim::Entity &_entity,
        const json &previous_state,
        float time_diff);

    /**
     * @brief Create a Connection object
     *
     * @param _entity
     * @param uri
     * @param thrusters_name
     * @return true
     * @return false
     */
    bool createConnection(
        const gz::sim::Entity &_entity,
        const std::string &_name,
        const sdf::ElementPtr _sdf);

    bool activateConnection(const gz::sim::Entity &_entity);

    bool deactivateConnection(const gz::sim::Entity &_entity);

    std::string getURI(const gz::sim::Entity &_entity);

protected:
    static std::shared_ptr<XdynWebsocket> m_instance;

    static std::mutex m_mutex;

private:
    bool send(const gz::sim::Entity &_entity, const std::string &message);

    void onMessage(
        websocketpp::connection_hdl hdl,
        websocketpp::config::asio_client::message_type::ptr msg);

    void onOpen(
        const gz::sim::Entity &_entity,
        websocketpp::connection_hdl hdl);

    void onFail(
        const gz::sim::Entity &_entity,
        websocketpp::connection_hdl hdl);

private:
    // Websocket stuff
    Client m_client;

    /**
     * @brief Thread to run client.
     *
     */
    Weblib::shared_ptr<Weblib::thread> m_thread;

    /**
     * @brief Mapping for entity and naming
     *
     */
    static std::unordered_map<gz::sim::Entity, std::string> m_name_mapping;

    /**
     * @brief Entity mapping
     *
     */
    static std::unordered_map<std::string, gz::sim::Entity> m_entity_mapping;

    /**
     * @brief URI mapping
     *
     */
    static std::unordered_map<gz::sim::Entity, std::string> m_uri;

    /**
     * @brief Connection mapping
     *
     */
    static std::unordered_map<gz::sim::Entity, Client::connection_ptr>
        m_connection_mapping;

    /**
     * @brief Connection mapping
     *
     */
    static std::unordered_map<Client::connection_ptr, gz::sim::Entity>
        m_connection_entity_mapping;

    /**
     * @brief Connection status mapping
     *
     */
    static std::unordered_map<gz::sim::Entity, std::string> m_status;

    /**
     * @brief Msg saving
     *
     */
    static std::unordered_map<gz::sim::Entity, std::mutex> m_msg_mutex;

    /**
     * @brief Thread locks
     *
     */
    static std::unordered_map<gz::sim::Entity, std::condition_variable>
        m_msg_cv;

    /**
     * @brief Save state of vessel
     *
     */
    static std::unordered_map<gz::sim::Entity, json> m_saved_state;
};
}  // namespace lotusim::gazebo
#endif