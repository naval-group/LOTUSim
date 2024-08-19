#include "world_plugins/TcpUdpInterface.h"

namespace liquidai {
namespace gazebo {

// TODO: create explosion pipeline. Create default function update for special
// commands

TcpUdpInterface::TcpUdpInterface()
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
            gzwarn << "Error closing UDP socket: " << "\n";
        }
    }

    // Close the TCP socket if it is open
    if (m_tcp_socket_ptr && m_tcp_socket_ptr->is_open()) {
        boost::system::error_code ec;
        m_tcp_socket_ptr->close(ec);
        if (ec) {
            // Handle error
            gzwarn << "Error closing TCP socket: " << ec.message() << "\n";
        }
    }
    // Stop the io_context to cancel any outstanding asynchronous operations
    m_io_context.stop();
}

bool TcpUdpInterface::ConfigureInterface(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    // defining endpoint
    m_udp_socket_ptr = std::make_shared<ip::udp::socket>(m_io_context);
    m_tcp_socket_ptr = std::make_shared<ip::tcp::socket>(m_io_context);
    std::string ip = "127.0.0.1";
    float udp_port = 23456;
    float tcp_port = 23457;
    if (sdfPtr->HasElement("ip")) {
        ip = sdfPtr->Get<std::string>("ip");
    }
    if (sdfPtr->HasElement("udp_port")) {
        udp_port = sdfPtr->Get<float>("udp_port");
    }
    if (sdfPtr->HasElement("tcp_port")) {
        tcp_port = sdfPtr->Get<float>("tcp_port");
    }

    // creating udp socket, no handling required
    gzmsg << "TcpUdpInterface: Creating UDP connection at ip:" << ip
          << "port: " << udp_port << "\n";
    m_udp_endpoint = ip::udp::endpoint(ip::address::from_string(ip), udp_port);
    m_udp_socket_ptr->open(ip::udp::v4());

    // creating tcp socket
    gzmsg << "TcpUdpInterface: Creating TCP connection at ip:" << ip
          << "port: " << tcp_port << "\n";
    m_tcp_endpoint = ip::tcp::endpoint(ip::address::from_string(ip), tcp_port);
    boost::system::error_code ec;
    m_tcp_socket_ptr->connect(m_tcp_endpoint, ec);

    if (ec) {
        gzwarn << "TcpUdpInterface: TCP Failed to connect to " << ip
               << " on port " << tcp_port << ". Error: " << ec.message()
               << "\n";
        // Todo: figure out whether to retry
    } else {
        gzmsg << "TcpUdpInterface: TCP Successfully connected to " << ip
              << " on port " << tcp_port << "\n";
    }
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
    std::string msgs = "{\"VesselsInfo\": [";
    for (auto &&temp : poses) {
        std::string vessel_name = temp.first;
        gz::math::Pose3d pose = temp.second;
        std::string msg = "{";
        msg += "\"name\": \"" + vessel_name + "\", ";
        msg += "\"time\": \"" + std::to_string(secs) + "\", ";
        msg += " \"position\": {";
        msg += " \"x\": " + std::to_string(pose.X()) + ", ";
        msg += " \"y\": " + std::to_string(pose.Y()) + ", ";
        msg += " \"z\": " + std::to_string(pose.Z());
        msg += "}, \"rotation\": {";
        msg += " \"x\": " + std::to_string(pose.Rot().X()) + ", ";
        msg += " \"y\": " + std::to_string(pose.Rot().Y()) + ", ";
        msg += " \"z\": " + std::to_string(pose.Rot().Z()) + ", ";
        msg += " \"w\": " + std::to_string(pose.Rot().W());
        msg += "}, \"thrusters\": [";
        for (auto &&cmd_msg : m_xdyn_cmd[vessel_name].cmd()) {
            msg += "{ \"name\": \"" + cmd_msg.name() + "\", ";
            msg += " \"rpm\": " + std::to_string(cmd_msg.rpm()) + "},";
        }
        if (msg.back() == ',') {
            msg.pop_back();
        }
        msg += "]},";
        msgs += msg;
    }
    if (msgs.back() == ',') {
        msgs.pop_back();
    }
    msgs.append("]}");
    auto sent = m_udp_socket_ptr->send_to(
        boost::asio::buffer(msgs),
        m_udp_endpoint,
        0,
        err);

    m_io_context.run_one();
    return true;
}

void TcpUdpInterface::ThrustCmdCB(const gz_liquidai_msgs::msgs::XdynCmd &_msg)
{
    std::lock_guard<std::mutex> lock(m_xdyn_mutex);
    gzmsg << "Received: " << _msg.name() << "\n";
    if (m_xdyn_cmd.find(_msg.name()) != m_xdyn_cmd.end()) {
        m_xdyn_cmd[_msg.name()] = std::move(_msg);
    } else {
        gzmsg << "Received unknown vessel: " << _msg.name() << "\n";
    }
}

void TcpUdpInterface::CollisionCB(const gz::msgs::Contacts &_msg)
{
    std::lock_guard<std::mutex> lock(m_collision_mutex);
    for (auto &&collision : _msg.contact()) {
        m_collisions.insert(std::move(collision.collision1().id()));
        m_collisions.insert(std::move(collision.collision2().id()));
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
    gzmsg << "RenderPlugin checking model: " << vessel_name << "\n";

    if (m_xdyn_cmd.find(vessel_name) == m_xdyn_cmd.end() &&
        sdfptr->HasElement("physics_server_interface")) {
        // Create thruster subscriber to render thrusters update
        gz_liquidai_msgs::msgs::XdynCmd xdyn_cmd;
        xdyn_cmd.set_name(vessel_name);
        auto sdfPtr_physics =
            sdfptr->GetElement("physics_server_interface")->GetFirstElement();
        do {
            if (sdfPtr_physics->HasElement("thrusters")) {
                auto sdfPtr_thrusters =
                    sdfPtr_physics->GetElement("thrusters")->GetFirstElement();
                do {
                    gz_liquidai_msgs::msgs::XdynCmd_ThrusterCmd *thruster_cmd =
                        xdyn_cmd.add_cmd();
                    thruster_cmd->set_name(
                        sdfPtr_thrusters->Get<std::string>());
                    thruster_cmd->set_rpm(0.0);
                    thruster_cmd->set_pd(0.79);
                    thruster_cmd->set_beta(0.0);
                    sdfPtr_thrusters = sdfPtr_thrusters->GetNextElement();
                } while (sdfPtr_thrusters != sdf::ElementPtr(nullptr));
            }
            sdfPtr_physics = sdfPtr_physics->GetNextElement();
        } while (sdfPtr_physics != sdf::ElementPtr(nullptr));
        std::lock_guard<std::mutex> lock(m_xdyn_mutex);
        m_xdyn_cmd[vessel_name] = std::move(xdyn_cmd);
        m_gz_node.Subscribe(
            vessel_name + "/cmd_vel",
            &TcpUdpInterface::ThrustCmdCB,
            this);
    }
    // If renderer fails to spawn vessel, no point adding model to list to
    // update
    return SendCreateMessage(vessel_name, pose, renderer_type_name);
}

bool TcpUdpInterface::DestroyVessel(const std::string &vessel_name)
{
    if (m_xdyn_cmd.find(vessel_name) != m_xdyn_cmd.end()) {
        m_xdyn_cmd.erase(vessel_name);
        m_gz_node.Unsubscribe(vessel_name + "/cmd_vel");
    }
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
}

bool TcpUdpInterface::SendCreateMessage(
    const std::string &name,
    const gz::math::Pose3d &pose,
    const std::string &type)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    gzmsg << "RenderPlugin creating model: " << name << "\n";
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"cmd\": \"create\", ";
    msg += "\"name\": \"" + name + "\", ";
    msg += "\"pose\": {";
    msg += "\"x\": " + std::to_string(pose.X()) + ", ";
    msg += "\"y\": " + std::to_string(pose.Y()) + ", ";
    msg += "\"z\": " + std::to_string(pose.Z());
    msg += "},";
    msg += "\"type\": \"" + type + "\"";
    msg += "}";

    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r\n");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        gzmsg << "Received ACK response\n";
        return true;
    } else {
        gzwarn << "Unexpected response: " << line << "\n";
        return false;
    }
}

bool TcpUdpInterface::SendDestroyMessage(const std::string &name)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    gzmsg << "RenderPlugin destroying model: " << name << "\n";
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"cmd\": \"destroy\", ";
    msg += "\"name\": \"" + name + "\"";
    msg += "}";

    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r\n");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        gzmsg << "Received ACK response\n";
        return true;
    } else {
        gzwarn << "Unexpected response: " << line << "\n";
        return false;
    }
}

bool TcpUdpInterface::SendExplodeMessage(const std::string &name)
{
    if (!CheckTcpOpen()) {
        return false;
    }

    gzmsg << "RenderPlugin exploding model: " << name << "\n";
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"cmd\": \"explode\", ";
    msg += "\"name\": \"" + name + "\"";
    msg += "}";

    boost::asio::write(*m_tcp_socket_ptr, boost::asio::buffer(msg));
    boost::asio::streambuf response;
    // To add timeout
    boost::asio::read_until(*m_tcp_socket_ptr, response, "\r\n");
    std::istream response_stream(&response);
    std::string line;
    std::getline(response_stream, line);
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    // Check if the response is "ACK"
    if (line == "ACK") {
        gzmsg << "Received ACK response\n";
        return true;
    } else {
        gzwarn << "Unexpected response: " << line << "\n";
        return false;
    }
}

}  // namespace gazebo
}  // namespace liquidai