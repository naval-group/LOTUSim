#ifndef __TCP_UDP_RENDER_INTERFACE_HH__
#define __TCP_UDP_RENDER_INTERFACE_HH__

#include <gz/msgs/contacts.pb.h>

#include <boost/asio.hpp>
#include <gz/sim/components/Name.hh>

#include "RenderInterface.hpp"
#include "gz/sim/Util.hh"

namespace lotusim::gazebo {

namespace ip = boost::asio::ip;

constexpr unsigned short DEFAULT_UDP_PORT = 23456;
constexpr unsigned short DEFAULT_TCP_PORT = 23457;

/**
 * @brief UDP interface for Renderer
 *
 */

class TcpUdpInterface : public RenderInterfaceBase {
public:
    TcpUdpInterface(
        const std::string &world_name,
        std::shared_ptr<spdlog::logger> logger);
    ~TcpUdpInterface();

    bool ConfigureInterface(const std::shared_ptr<const sdf::Element> &_sdf);

    bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses);

    /**
     * @brief Method to create new vessel
     * Handles reading of vessel params, and how the renderer create the vessel
     *
     * @param vessel_name
     * @param type_name
     * @return true
     * @return false
     */
    bool CreateVessel(
        const std::string &vessel_name,
        const gz::math::Pose3d &pose,
        sdf::ElementPtr sdfptr);

    bool DestroyVessel(const std::string &vessel_name);

    virtual bool CustomPreUpdates(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    virtual bool CustomUpdates(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    void CollisionCB(const gz::msgs::Contacts &_msg);

    bool CheckTcpOpen();

    bool SendCreateMessage(
        const std::string &name,
        const gz::math::Pose3d &pose,
        const std::string &type);

    bool SendDestroyMessage(const std::string &name);

    bool SendExplodeMessage(const std::string &name);

private:
    boost::asio::io_context m_io_context;
    std::shared_ptr<ip::udp::socket> m_udp_socket_ptr;
    ip::udp::endpoint m_udp_endpoint;

    std::shared_ptr<ip::tcp::socket> m_tcp_socket_ptr;
    ip::tcp::endpoint m_tcp_endpoint;

    std::mutex m_collision_mutex;
    std::unordered_set<gz::sim::Entity> m_collisions;

    std::unordered_set<gz::sim::Entity> m_remove_queue;
};

}  // namespace lotusim::gazebo
#endif