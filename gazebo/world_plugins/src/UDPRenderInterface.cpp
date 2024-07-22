
#include "world_plugins/UDPRenderInterface.h"

bool UDPRenderInterface::ConfigureInterface(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    // defining endpoint
    m_socket_ptr = std::make_shared<ip::udp::socket>(m_io_context);
    std::string ip = "127.0.0.1";
    float port = 23456;
    if (sdfPtr->HasElement("ip")) {
        ip = sdfPtr->Get<std::string>("ip");
    }
    if (sdfPtr->HasElement("port")) {
        port = sdfPtr->Get<float>("port");
    }

    gzmsg << "UDPRenderInterface: Creating connection at ip:" << ip
          << "port: " << port << "\n";
    m_endpoint = ip::udp::endpoint(ip::address::from_string(ip), port);
    m_socket_ptr->open(ip::udp::v4());
}

bool UDPRenderInterface::SendPosition(
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
        msg.pop_back();
        msg += "]},";
        msgs += msg;
    }
    msgs.pop_back();
    msgs.append("]}");

    auto sent =
        m_socket_ptr->send_to(boost::asio::buffer(msgs), m_endpoint, 0, err);
    std::cout << msgs << std::endl;

    m_io_context.run_one();
    return true;
}

void UDPRenderInterface::ThrustCmd(const gz_liquidai_msgs::msgs::XdynCmd &_msg)
{
    std::cout << "Received: " << _msg.name() << std::endl;
    if (m_xdyn_cmd.find(_msg.name()) != m_xdyn_cmd.end()) {
        m_xdyn_cmd[_msg.name()] = std::move(_msg);
    }
    else {
        std::cout << "Received unknown vessel: " << _msg.name() << std::endl;
    }
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
bool UDPRenderInterface::CreateVessel(
    const std::string &vessel_name, sdf::ElementPtr sdfptr)
{
    std::string unity_type_name = "";
    if (sdfptr->HasElement("unity_type_name")) {
        unity_type_name = sdfptr->Get<std::string>("unity_type_name");
    }
    gzmsg << "RenderPlugin checking model: " << vessel_name << "\n";

    if (m_xdyn_cmd.find(vessel_name) == m_xdyn_cmd.end()) {
        // create m_xdyn_cmd
        // Main goal is to get thruster name:

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
        m_xdyn_cmd[vessel_name] = std::move(xdyn_cmd);
        m_gz_node.Subscribe(
            vessel_name + "/cmd_vel", &UDPRenderInterface::ThrustCmd, this);
    }
    // SendCreateMessage(vessel_name, unity_type_name);
};

bool UDPRenderInterface::DestroyVessel(const std::string &vessel_name)
{
    if (m_xdyn_cmd.find(vessel_name) != m_xdyn_cmd.end()) {
        m_xdyn_cmd.erase(vessel_name);
        m_gz_node.Unsubscribe(vessel_name + "/cmd_vel");
    }
    // SendDestroyMessage(vessel_name);
};

// Not tested
bool UDPRenderInterface::SendCreateMessage(std::string name, std::string type)
{
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"cmd\": \"create\", ";
    msg += "\"name\": \"" + name + "\", ";
    msg += "\"type\": \"" + type + "\"";
    msg += "}";

    auto sent =
        m_socket_ptr->send_to(boost::asio::buffer(msg), m_endpoint, 0, err);

    m_io_context.run_one();
    return true;
}

// Not tested
bool UDPRenderInterface::SendDestroyMessage(std::string name)
{
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"cmd\": \"destroy\", ";
    msg += "\"name\": \"" + name + "\"";
    msg += "}";

    auto sent =
        m_socket_ptr->send_to(boost::asio::buffer(msg), m_endpoint, 0, err);

    m_io_context.run_one();
    return true;
}