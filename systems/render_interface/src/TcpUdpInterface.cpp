#include "TcpUdpInterface.hpp"

namespace lotusim::gazebo {

// TODO: create explosion pipeline. Create default function update for special
// commands

TcpUdpInterface::TcpUdpInterface(
    const std::string &world_name,
    std::shared_ptr<spdlog::logger> logger)
    : RenderInterfaceBase(world_name, std::move(logger))
{
    m_gz_node.Subscribe("/collision", &TcpUdpInterface::CollisionCB, this);
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

bool TcpUdpInterface::ConfigureInterface(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
    // defining endpoint
    m_udp_socket_ptr = std::make_shared<ip::udp::socket>(m_io_context);
    m_tcp_socket_ptr = std::make_shared<ip::tcp::socket>(m_io_context);
    std::string ip = "127.0.0.1";
    unsigned short udp_port = DEFAULT_UDP_PORT;
    unsigned short tcp_port = DEFAULT_TCP_PORT;
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
        "TcpUdpInterface::ConfigureInterface : Creating UDP connection at ip: {} port: {}",
        ip,
        udp_port);
    m_udp_endpoint = ip::udp::endpoint(ip::address::from_string(ip), udp_port);
    m_udp_socket_ptr->open(ip::udp::v4());

    // creating tcp socket
    m_logger->info(
        "TcpUdpInterface::ConfigureInterface : Creating TCP connection at ip: {}  tcp_port: {}",
        ip,
        tcp_port);
    m_tcp_endpoint = ip::tcp::endpoint(ip::address::from_string(ip), tcp_port);
    boost::system::error_code ec;
    m_tcp_socket_ptr->connect(m_tcp_endpoint, ec);
    if (ec) {
        m_logger->warn(
            "TcpUdpInterface::ConfigureInterface : TCP Failed to connect to {} on port {}. Error: {}",
            ip,
            tcp_port,
            ec.message());

        // Todo: figure out whether to retry
    } else {
        m_logger->info(
            "TcpUdpInterface::ConfigureInterface : TCP Successfully connected to {}  on port {}",
            ip,
            tcp_port);
    }
    return true;
}

bool TcpUdpInterface::SendPosition(
    const std::chrono::steady_clock::duration &runTime,
    const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses)
{
    if (poses.empty()) {
        return true;
    }
    float secs =
        std::chrono::duration_cast<std::chrono::duration<float>>(runTime)
            .count();
    boost::system::error_code err;
    std::string msgs = R"({"VesselsInfo": [)";
    for (auto &&temp : poses) {
        std::string vessel_name = temp.first;
        gz::math::Pose3d pose = temp.second;

        // Construct the vessel message
        std::string msg = fmt::format(
            R"(
        {{
            "name": "{}",
            "time": "{}",
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
            secs,
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
    auto sent = m_udp_socket_ptr->send_to(
        boost::asio::buffer(msgs),
        m_udp_endpoint,
        0,
        err);

    m_io_context.run_one();
    return true;
}

void TcpUdpInterface::CollisionCB(const gz::msgs::Contacts &_msg)
{
    std::lock_guard<std::mutex> lock(m_collision_mutex);
    for (auto &&collision : _msg.contact()) {
        m_collisions.insert(collision.collision1().id());
        m_collisions.insert(collision.collision2().id());
    }
}

bool TcpUdpInterface::CheckTcpOpen()
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
bool TcpUdpInterface::CreateVessel(
    const std::string &vessel_name,
    const gz::math::Pose3d &pose,
    sdf::ElementPtr sdfptr)
{
    std::string renderer_type_name = "";
    if (sdfptr->HasElement("renderer_type_name")) {
        renderer_type_name = sdfptr->Get<std::string>("renderer_type_name");
    }
    m_logger->info("RenderPlugin checking model: {}", vessel_name);

    // If renderer fails to spawn vessel, no point adding model to list to
    // update
    return SendCreateMessage(vessel_name, pose, renderer_type_name);
}

bool TcpUdpInterface::DestroyVessel(const std::string &vessel_name)
{
    SendDestroyMessage(vessel_name);
    // Always true as we will still remove models from updating list
    return true;
}
bool TcpUdpInterface::CustomPreUpdates(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    for (auto &&entity : m_remove_queue) {
        _ecm.RequestRemoveEntity(entity);
    }
    return true;
}

bool TcpUdpInterface::CustomUpdates(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Collision handling
    std::lock_guard<std::mutex> lock(m_collision_mutex);
    std::unordered_set<gz::sim::Entity> _collided_entity;
    for (auto &&collision_entity : m_collisions) {
        gz::sim::Entity _model_entity =
            gz::sim::topLevelModel(collision_entity, _ecm);
        if (_model_entity) {
            _collided_entity.insert(_model_entity);
        }
    }
    m_collisions.clear();
    for (const gz::sim::Entity &collision_model : _collided_entity) {
        auto name_opt =
            _ecm.Component<gz::sim::components::Name>(collision_model);
        if (name_opt) {
            SendExplodeMessage(name_opt->Data());
        }
        m_remove_queue.insert(collision_model);
    }
    return true;
}

bool TcpUdpInterface::SendCreateMessage(
    const std::string &name,
    const gz::math::Pose3d &pose,
    const std::string &type)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin creating model: {}", name);
    boost::system::error_code err;
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

bool TcpUdpInterface::SendDestroyMessage(const std::string &name)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin destroying model: {}", name);
    boost::system::error_code err;
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

bool TcpUdpInterface::SendExplodeMessage(const std::string &name)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    m_logger->info("RenderPlugin exploding model: {}", name);
    boost::system::error_code err;
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