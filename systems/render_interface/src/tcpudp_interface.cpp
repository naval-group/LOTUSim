/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "render_interface/tcpudp_interface.hpp"

namespace lotusim::gazebo {

// TODO: create explosion pipeline. Create default function update for special
// commands

TcpUdpInterface::TcpUdpInterface(
    const std::string& world_name,
    std::shared_ptr<spdlog::logger> logger)
    : RenderInterfaceBase(world_name, std::move(logger))
{
    m_logger = logger::createConsoleAndFileLogger(
        "tcp_render_interface",
        "tcp_render_interface.txt");
}

TcpUdpInterface::~TcpUdpInterface()
{
    if (m_udp_socket_ptr && m_udp_socket_ptr->is_open()) {
        boost::system::error_code ec;
        m_udp_socket_ptr->close(ec);
        if (ec) {
            // Handle error
            m_logger->warn(
                "TcpUdpInterface::~TcpUdpInterface: Error closing UDP socket");
        }
    }

    // Close the TCP socket if it is open
    if (m_tcp_socket_ptr && m_tcp_socket_ptr->is_open()) {
        boost::system::error_code ec;
        m_tcp_socket_ptr->close(ec);
        if (ec) {
            // Handle error
            m_logger->warn(
                "TcpUdpInterface::~TcpUdpInterface: Error closing TCP socket: {}",
                ec.message());
        }
    }
    // Stop the io_context to cancel any outstanding asynchronous operations
    m_io_context.stop();
}

bool TcpUdpInterface::configureInterface(
    const std::shared_ptr<const sdf::Element>& _sdf)
{
    // defining endpoint
    m_udp_socket_ptr = std::make_shared<ip::udp::socket>(m_io_context);
    m_tcp_socket_ptr = std::make_shared<ip::tcp::socket>(m_io_context);

    std::string ip = "127.0.0.1";
    float udp_port = DEFAULT_UDP_PORT;  // e.g., 23456
    float tcp_port = DEFAULT_TCP_PORT;  // e.g., 23457

    if (_sdf->HasElement("ip")) {
        ip = _sdf->Get<std::string>("ip");
    }
    if (_sdf->HasElement("udp_port")) {
        udp_port = _sdf->Get<float>("udp_port");
    }
    if (_sdf->HasElement("tcp_port")) {
        tcp_port = _sdf->Get<float>("tcp_port");
    }

    // creating udp socket, no handling required
    m_logger->info(
        "TcpUdpInterface::configureInterface : Creating UDP connection at ip: {} port: {}",
        ip,
        udp_port);
    try {
        m_udp_endpoint =
            ip::udp::endpoint(ip::address::from_string(ip), udp_port);
        m_udp_socket_ptr->open(ip::udp::v4());
    } catch (const boost::system::system_error& e) {
        m_logger->error("Failed to open UDP socket: {}", e.what());
        return false;
    }

    // creating tcp socket
    m_logger->info(
        "TcpUdpInterface::configureInterface : Creating TCP connection at ip: {}  tcp_port: {}",
        ip,
        tcp_port);
    m_tcp_endpoint = ip::tcp::endpoint(ip::address::from_string(ip), tcp_port);
    boost::system::error_code ec;
    m_tcp_socket_ptr->connect(m_tcp_endpoint, ec);

    if (ec) {
        m_logger->warn(
            "TcpUdpInterface::configureInterface : TCP Failed to connect to {} on port {}. Error: {}",
            ip,
            tcp_port,
            ec.message());
        return false;
        // Todo: figure out whether to retry
    }
    m_logger->info(
        "TcpUdpInterface::configureInterface : TCP Successfully connected to {}  on port {}",
        ip,
        tcp_port);

    return true;
}

bool TcpUdpInterface::sendPosition(
    const std::chrono::steady_clock::duration& runTime,
    const std::vector<std::pair<std::string, gz::math::Pose3d>>& poses)
{
    float secs =
        std::chrono::duration_cast<std::chrono::duration<float>>(runTime)
            .count();
    boost::system::error_code err;
    std::string msgs =
        R"({"time": )" + std::to_string(secs) + R"(, "VesselsInfo": [)";
    for (auto&& temp : poses) {
        std::string vessel_name = temp.first;
        gz::math::Pose3d pose = temp.second;

        // Construct the vessel message
        std::string msg = fmt::format(
            R"(
        {{
            "name": "{}",
            "position": {{
                "x": {},
                "y": {},
                "z": {}
            }},
            "rotation": {{
                "x": {},
                "y": {},
                "z": {},
                "w": {}
            }},
            "thrusters": [
                    )",
            vessel_name,
            pose.X(),
            pose.Y(),
            pose.Z(),
            pose.Rot().X(),
            pose.Rot().Y(),
            pose.Rot().Z(),
            pose.Rot().W());

        msg += R"(]},)";
        msgs += msg;
    }

    // Remove the trailing comma and finalize the JSON structure
    if (!msgs.empty() && msgs.back() == ',') {
        msgs.pop_back();
    }
    msgs += "]}";
    m_udp_socket_ptr
        ->send_to(boost::asio::buffer(msgs), m_udp_endpoint, 0, err);

    m_io_context.run_one();
    return true;
}

bool TcpUdpInterface::checkTcpOpen()
{
    if (m_tcp_socket_ptr && m_tcp_socket_ptr->is_open()) {
        boost::system::error_code ec;
        m_tcp_socket_ptr->remote_endpoint(ec);
        if (ec) {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

/**
 * @brief Create subscriber for thruster command and update Unity to render
 * new vessel
 *
 * @param vessel_name
 * @param type_name
 * @return true
 * @return false
 */
bool TcpUdpInterface::createVessel(
    const std::string& vessel_name,
    const gz::math::Pose3d& pose,
    sdf::ElementPtr sdfptr)
{
    std::string renderer_type_name = "";
    if (sdfptr->HasElement("renderer_type_name")) {
        renderer_type_name = sdfptr->Get<std::string>("renderer_type_name");
    }
    m_logger->info("RenderPlugin checking model: {}", vessel_name);

    // If renderer fails to spawn vessel, no point adding model to list to
    // update
    return sendCreateMessage(vessel_name, pose, renderer_type_name);
}

bool TcpUdpInterface::destroyVessel(const std::string& vessel_name)
{
    sendDestroyMessage(vessel_name);
    // Always true as we will still remove models from updating list
    return true;
}
bool TcpUdpInterface::customPreUpdates(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    return true;
}

bool TcpUdpInterface::customUpdates(
    const gz::sim::UpdateInfo&,
    const gz::sim::EntityComponentManager&)
{
    return true;
}

bool TcpUdpInterface::sendCreateMessage(
    const std::string& name,
    const gz::math::Pose3d& pose,
    const std::string& type)
{
    if (!checkTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin creating model: {}", name);
    std::string msg = fmt::format(
        R"({{
        "cmd": "create",
        "name": "{}",
        "pose": {{
            "x": {},
            "y": {},
            "z": {}
        }},
        "type": "{}"
    }})",
        name,
        pose.X(),
        pose.Y(),
        pose.Z(),
        type);
    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        m_logger->info("Received ACK response");
        return true;
    } else {
        m_logger->warn("Unexpected response: {}", line);
        return false;
    }
}

bool TcpUdpInterface::sendDestroyMessage(const std::string& name)
{
    if (!checkTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin destroying model: {}", name);
    std::string msg = fmt::format(
        R"(
        {
            "cmd": "destroy",
            "name": "{}"
        })",
        name);

    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        m_logger->info("Received ACK response");
        return true;
    } else {
        m_logger->warn("Unexpected response: {}", line);
        return false;
    }
}

bool TcpUdpInterface::sendExplodeMessage(const std::string& name)
{
    if (!checkTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin exploding model: {}", name);
    std::string msg = fmt::format(
        R"(
        {
            "cmd": "explode",
            "name": "{}"
        })",
        name);

    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        m_logger->info("Received ACK response");
        return true;
    } else {
        m_logger->warn("Unexpected response: {}", line);
        return false;
    }
}

}  // namespace lotusim::gazebo