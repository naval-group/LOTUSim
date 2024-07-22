#ifndef __UDP_RENDER_INTERFACE_HH__
#define __UDP_RENDER_INTERFACE_HH__

#include "gz_liquidai_msgs/msgs/xdyncmdmsg.pb.h"
#include "world_plugins/RenderInterface.h"
#include <boost/asio.hpp>

namespace liquidai {
namespace gazebo {

namespace ip = boost::asio::ip;

/**
 * @brief UDP interface for Unity
 *
 */
class UDPRenderInterface : public RenderInterfaceBase {
public:
    bool ConfigureInterface(const std::shared_ptr<const sdf::Element> &_sdf);

    bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses);

    void ThrustCmd(const gz_liquidai_msgs::msgs::XdynCmd &_msg);

    /**
     * @brief Method to create new vessel
     * Handles reading of vessel params, subscribing to thruster topics
     * and updating Unity about vessel creation
     *
     * @param vessel_name
     * @param type_name
     * @return true
     * @return false
     */
    bool CreateVessel(const std::string &vessel_name, sdf::ElementPtr sdfptr);

    bool DestroyVessel(const std::string &vessel_name);

    // Not tested
    bool SendCreateMessage(std::string name, std::string type);

    // Not tested
    bool SendDestroyMessage(std::string name);

private:
    boost::asio::io_context m_io_context;
    std::shared_ptr<ip::udp::socket> m_socket_ptr;
    ip::udp::endpoint m_endpoint;

    /**
     * @brief Mapping from vessel name to cmd
     *
     */
    std::unordered_map<std::string, gz_liquidai_msgs::msgs::XdynCmd> m_xdyn_cmd;
};

} // namespace gazebo
} // namespace liquidai
#endif