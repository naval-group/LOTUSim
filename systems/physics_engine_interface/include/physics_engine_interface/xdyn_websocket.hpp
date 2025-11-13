/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_XDYN_WEBSOCKET_HH_
#define LOTUSIM_XDYN_WEBSOCKET_HH_

#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "physics_engine_interface/physics_interface_base.hpp"

namespace lotusim::gazebo {

namespace Weblib = websocketpp::lib;
using hdl = websocketpp::connection_hdl;
using Client = websocketpp::client<websocketpp::config::asio_client>;
using json = nlohmann::json;

static const gz::math::Quaterniond
    q_ned_to_enu(0.0, 0.70710678118, 0.70710678118, 0.0);

gz::math::Quaterniond quatNedToEnu(const gz::math::Quaterniond& q_ned);
gz::math::Quaterniond quatEnuToNed(const gz::math::Quaterniond& q_enu);
gz::math::Vector3d vecNedToEnu(const gz::math::Vector3d& v_ned);
gz::math::Vector3d vecEnuToNed(const gz::math::Vector3d& v_enu);

constexpr unsigned short DEFAULT_WEBSOCKET_TIMEOUT = 5;

/**
 * @class XDynWebSocketClient
 * @brief Singleton class for connecting to XDyn through WebSocket.
 *
 * This singleton class manages communication with XDyn using WebSocket.
 * It allows reuse of the WebSocket client to send requests and receive
 * responses.
 *
 * Upon sending a request to XDyn and receiving a message, a frame conversion
 * is performed. XDyn uses the NED (North-East-Down) coordinate convention and
 * the right-hand axis convention.
 *
 * ### Request Format
 * Requests to XDyn must be made in JSON format containing the following fields:
 * ```
 * {
 *   "t":  <time>,
 *   "x":  <position_x>,
 *   "y":  <position_y>,
 *   "z":  <position_z>,
 *   "u":  <velocity_x>,
 *   "v":  <velocity_y>,
 *   "w":  <velocity_z>,
 *   "p":  <angular_rate_x>,
 *   "q":  <angular_rate_y>,
 *   "r":  <angular_rate_z>,
 *   "qr": <quaternion_r>,
 *   "qi": <quaternion_i>,
 *   "qj": <quaternion_j>,
 *   "qk": <quaternion_k>
 * }
 * ```
 *
 * ### Example Configuration
 * ```xml
 * <lotusim_param>
 *    <physics_engine_interface>
 *        <surface>
 *            <connection_type>XDynWebSocket</connection_type>
 *            <uri>ws://127.0.0.1:12345</uri>
 *            <thrusters>
 *                <thruster1>PSPropRudd</thruster1>
 *                <thruster2>SBPropRudd</thruster2>
 *            </thrusters>
 *        </surface>
 *        <init_state>Surface</init_state>
 *    </physics_engine_interface>
 * </lotusim_param>
 * ```
 *
 * ### Example Command Map
 * The `m_vessels_cmd_map_ptr` string is formatted as:
 * ```json
 * {
 *   "PSPropRudd(P/D)": 0.79,
 *   "PSPropRudd(beta)": 0.0,
 *   "PSPropRudd(rpm)": 0.0,
 *   "SBPropRudd(P/D)": 0.79,
 *   "SBPropRudd(beta)": 0.0,
 *   "SBPropRudd(rpm)": 0.0
 * }
 * ```
 *
 * @note The NED frame convention is used throughout all communications.
 */

class XdynWebsocket : public PhysicsInterfaceBase {
public:
    XdynWebsocket();

    ~XdynWebsocket();

    void operator=(const XdynWebsocket&) = delete;

    XdynWebsocket(XdynWebsocket& other) = delete;

    static std::shared_ptr<XdynWebsocket> getInstance(
        const gz::sim::Entity& _entity,
        const std::string& _name);

    /**
     * @brief Get the New State object using given vessel state
     *
     * @param _entity
     * @param previous_state
     * @param time_dif
     * @return std::optional<json>
     */
    std::optional<std::tuple<VesselInformation, DomainType>> getNewState(
        const gz::sim::Entity& _entity,
        const VesselInformation& previous_state,
        float time_diff) override final;

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
        const gz::sim::Entity& _entity,
        const std::string& _name,
        const sdf::ElementPtr _sdf) override final;

    bool removeConnection(const gz::sim::Entity& _entity) override final;

    bool activateConnection(const gz::sim::Entity& _entity) override final;

    bool deactivateConnection(const gz::sim::Entity& _entity) override final;

    std::string getURI(const gz::sim::Entity& _entity) override final;

protected:
    static std::shared_ptr<XdynWebsocket> m_instance;

    static std::mutex m_instance_mutex;

private:
    bool send(const gz::sim::Entity& _entity, const std::string& message);

    void onMessage(
        websocketpp::connection_hdl hdl,
        websocketpp::config::asio_client::message_type::ptr msg);

    void onOpen(
        const gz::sim::Entity& _entity,
        websocketpp::connection_hdl hdl);

    void onFail(
        const gz::sim::Entity& _entity,
        websocketpp::connection_hdl hdl);

private:
    // Websocket stuff
    Client m_client;

    /**
     * @brief Thread to run client.
     *
     */
    Weblib::shared_ptr<Weblib::thread> m_thread;

    static std::mutex m_variable_mutex;

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
     * @brief Mapping of entity to URI of the physics engine
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
    static std::unordered_map<gz::sim::Entity, VesselInformation> m_saved_state;
};
}  // namespace lotusim::gazebo
#endif