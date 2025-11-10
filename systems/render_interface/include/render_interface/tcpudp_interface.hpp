/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_TCP_UDP_RENDER_INTERFACE_HPP_
#define LOTUSIM_TCP_UDP_RENDER_INTERFACE_HPP_

#include <boost/asio.hpp>
#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include "render_interface/render_interface.hpp"

namespace lotusim::gazebo {

namespace ip = boost::asio::ip;

constexpr unsigned short DEFAULT_UDP_PORT = 23456;
constexpr unsigned short DEFAULT_TCP_PORT = 23457;

/**
 * @brief TCPUDP interface for Renderer
 *
 * <plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">
 *  <connection_protocol>TCPUDP</connection_protocol>
 *  <ip>127.0.0.1</ip>
 *  <udp_port>23456</udp_port>   <!-- For position updates -->
 *  <tcp_port>23457</tcp_port>   <!-- For control commands -->
 * </plugin>
 */

class TcpUdpInterface final : public RenderInterfaceBase {
public:
    TcpUdpInterface(
        const std::string& world_name,
        std::shared_ptr<spdlog::logger> logger);

    ~TcpUdpInterface() override;

    bool configureInterface(
        const std::shared_ptr<const sdf::Element>& _sdf) override;

    /**
     * @brief Inherited sendPosition method
     *
     * @param runTime
     * @param poses
     * @return true
     * @return false
     */
    bool sendPosition(
        const std::chrono::steady_clock::duration& runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>>& poses)
        override;

    /**
     * @brief Method to create new vessel
     * Handles reading of vessel params, and how the renderer create the vessel
     *
     * @param vessel_name
     * @param type_name
     * @return true
     * @return false
     */
    bool createVessel(
        const std::string& vessel_name,
        const gz::math::Pose3d& pose,
        sdf::ElementPtr sdfptr) override;

    bool destroyVessel(const std::string& vessel_name) override;

    virtual bool customPreUpdates(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

    virtual bool customUpdates(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief Check if TCP connection is open
     *
     * @return true if TCP socket is open
     * @return false otherwise
     */
    bool checkTcpOpen();

    /**
     * @brief Send vessel creation message via TCP
     *
     * @param name Vessel name
     * @param pose Initial pose
     * @param type Vessel type
     * @return true if send successful
     * @return false otherwise
     */
    bool sendCreateMessage(
        const std::string& name,
        const gz::math::Pose3d& pose,
        const std::string& type);

    /**
     * @brief Send vessel destruction message via TCP
     *
     * @param name Vessel name
     * @return true if send successful
     * @return false otherwise
     */
    bool sendDestroyMessage(const std::string& name);

    /**
     * @brief Send explosion effect message via TCP
     *
     * @param name Vessel name
     * @return true if send successful
     * @return false otherwise
     */
    bool sendExplodeMessage(const std::string& name);

private:
    /**
     * @brief Boost ASIO IO context
     *
     */
    boost::asio::io_context m_io_context;

    /**
     * @brief UDP socket for position updates
     *
     */
    std::shared_ptr<ip::udp::socket> m_udp_socket_ptr;

    /**
     * @brief UDP endpoint
     *
     */
    ip::udp::endpoint m_udp_endpoint;

    /**
     * @brief TCP socket for commands
     *
     */
    std::shared_ptr<ip::tcp::socket> m_tcp_socket_ptr;

    /**
     * @brief TCP endpoint
     *
     */
    ip::tcp::endpoint m_tcp_endpoint;
};

}  // namespace lotusim::gazebo
#endif