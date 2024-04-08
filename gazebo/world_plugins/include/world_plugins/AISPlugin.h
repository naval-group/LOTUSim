#ifndef __AIS_PLUGIN_HH__
#define __AIS_PLUGIN_HH__

#include "gz_liquidai_msgs/msg/aismsg.pb.h"
#include "world_plugins/CoordinateSystemUtil.h"

#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <boost/asio.hpp>
#include <chrono>
#include <string.h>

namespace liquidai {
namespace gazebo {
    
namespace ip = boost::asio::ip;

/**
 * @brief This is a plugin for AIS array publisher.
 * All vessel in the world should contains the word vessel in name.
 *
 * <plugin filename="ais_plugin" name="liquidai::gazebo::AISPlugin">
 *      <period> 1 </period>
 * </plugin>
 */
class AISPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate {

public:
    AISPlugin();
    
    ~AISPlugin();
    
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:

    /**
     * Send position to Unity
    */
    void SendPosition();

    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    gz::transport::Node::Publisher m_ais_pub;
    std::shared_ptr<gz::transport::Node> m_gz_node;
    std::vector<gz::sim::Entity> m_vessel_entities;

    boost::asio::io_context m_io_context;
    std::shared_ptr<ip::udp::socket> m_socket_ptr;
    ip::udp::endpoint m_endpoint;

    std::vector<gz::sim::Entity> m_vessel_entities_pose;
    std::vector<std::pair<std::string, gz::math::Pose3d>> m_pose;
};

} // namespace gazebo
} // namespace liquidai
#endif