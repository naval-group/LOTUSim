#ifndef __UDP_RENDER_INTERFACE_HH__
#define __UDP_RENDER_INTERFACE_HH__

#include "world_plugins/RenderInterface.h"
#include <boost/asio.hpp>

namespace liquidai {
namespace gazebo {

namespace ip = boost::asio::ip;

class UDPRenderInterface : public RenderInterfaceBase {
public:
    bool ConfigureInterface(const std::shared_ptr<const sdf::Element> &_sdf)
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
        std::string msgs = "{\"poses\": [";
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
            msg += "}},";
            msgs += msg;
        }
        msgs.pop_back();
        msgs.append("]}");

        auto sent = m_socket_ptr->send_to(
            boost::asio::buffer(msgs), m_endpoint, 0, err);
        // std::cout << msgs << std::endl;

        m_io_context.run_one();
        return true;
    }

private:
    boost::asio::io_context m_io_context;
    std::shared_ptr<ip::udp::socket> m_socket_ptr;
    ip::udp::endpoint m_endpoint;
};

} // namespace gazebo
} // namespace liquidai
#endif