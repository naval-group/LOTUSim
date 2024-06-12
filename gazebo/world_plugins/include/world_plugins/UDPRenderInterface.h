#ifndef __UDP_RENDER_INTERFACE_HH__
#define __UDP_RENDER_INTERFACE_HH__

#include "gz_liquidai_msgs/msgs/xdyncmdmsg.pb.h"
#include "world_plugins/RenderInterface.h"
#include <boost/asio.hpp>

namespace liquidai {
namespace gazebo {

namespace ip = boost::asio::ip;

/**
 * @brief AHOY.
 * Testing idea of generic interface.
 * For developers who develop their own plugins, they should interface their
 * plugins here in their own custom interfaces.
 * ONLY when the system interfaces their plugins, then the system will put the
 * pub in the renderPlugin
 *
 */

class UDPRenderInterface : public RenderInterfaceBase {
public:
    bool ConfigureInterface(
        const std::shared_ptr<const sdf::Element> &_sdf,
        const std::shared_ptr<gz::transport::Node> _node)
    {
        m_gz_node = _node;
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

    bool SendPosition(
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
            float _rpm = 0.0;
            if (m_xdyn_cmd.find(temp.first) == m_xdyn_cmd.end()) {
                // create m_xdyn_cmd
                gz_liquidai_msgs::msgs::XdynCmd xdyn_cmd;
                xdyn_cmd.set_name(temp.first);
                // TODO Only 1 thruster for now.
                gz_liquidai_msgs::msgs::XdynCmd_ThrusterCmd *thruster_cmd =
                    xdyn_cmd.add_cmd();
                thruster_cmd->set_name("only_thruster");
                thruster_cmd->set_rpm(0.01);
                thruster_cmd->set_pd(0.79);
                thruster_cmd->set_beta(0.0);
                m_xdyn_cmd[temp.first] = std::move(xdyn_cmd);

                // Find if topic exist, if exist sub
                // std::vector<std::string> _topics;
                // m_gz_node->TopicList(_topics);
                // auto it = std::find_if(
                //     _topics.begin(),
                //     _topics.end(),
                //     [&temp](const std::string &sentence) {
                //         return sentence.find(temp.first + "/cmd_topic") !=
                //                std::string::npos;
                //     });

                // if (it != _topics.end()) {
                //     std::cout << "Substring found in sentence: " << *it
                //               << std::endl;
                //     m_gz_node->Subscribe(
                //         *it, &UDPRenderInterface::thrustCmd, this);
                // }
            }
            else {
                // get whether rpm is true or false
                auto cmd = m_xdyn_cmd[temp.first].mutable_cmd(0);
                _rpm = cmd->rpm();
            }

            std::string vessel_name = temp.first;
            gz::math::Pose3d pose = temp.second;
            std::string msg = "{";
            msg += "\"name\": \"" + vessel_name + "\", ";
            msg += "\"time\": \"" + std::to_string(secs) + "\", ";
            msg += "\"rpm\": \"" + std::to_string(_rpm) + "\", ";
            msg += " \"position\": {";
            msg += " \"x\": " + std::to_string(pose.X()) + ", ";
            msg += " \"y\": " + std::to_string(pose.Y()) + ", ";
            msg += " \"z\": " + std::to_string(pose.Z());
            msg += "}, \"rotation\": {";
            msg += " \"x\": " + std::to_string(pose.Rot().X()) + ", ";
            msg += " \"y\": " + std::to_string(pose.Rot().Y()) + ", ";
            msg += " \"z\": " + std::to_string(pose.Rot().Z()) + ", ";
            msg += " \"w\": " + std::to_string(pose.Rot().W());
            msg += "}},";
            msgs += msg;
        }
        msgs.pop_back();
        msgs.append("]}");

        auto sent = m_socket_ptr->send_to(
            boost::asio::buffer(msgs), m_endpoint, 0, err);
        std::cout << msgs << std::endl;

        m_io_context.run_one();
        return true;
    }

    void thrustCmd(const gz_liquidai_msgs::msgs::XdynCmd &_msg)
    {
        std::cout << "Received: " << _msg.name() << std::endl;
        if (m_xdyn_cmd.find(_msg.name()) != m_xdyn_cmd.end()) {
            m_xdyn_cmd[_msg.name()] = std::move(_msg);
        }
        else {
            std::cout << "Received unknown vessel: " << _msg.name()
                      << std::endl;
        }
    }

private:
    boost::asio::io_context m_io_context;
    std::shared_ptr<ip::udp::socket> m_socket_ptr;
    ip::udp::endpoint m_endpoint;

    std::shared_ptr<gz::transport::Node> m_gz_node;
    std::unordered_map<std::string, gz_liquidai_msgs::msgs::XdynCmd> m_xdyn_cmd;
};

} // namespace gazebo
} // namespace liquidai
#endif