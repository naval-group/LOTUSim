#ifndef __WAVE_RAO_HH__
#define __WAVE_RAO_HH__

#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

// #include "world_plugins/ManualRAO.h"
// #include "world_plugins/XdynGrpc.h"
#include "world_plugins/XdynWebSocket.h"

namespace liquidai {
namespace gazebo {

/**
 * @brief This is a plugin for wave RAO using external physics library
 * All vessel in the world should contains the word vessel in name.
 *
 * <plugin filename="ais_plugin" name="liquidai::gazebo::WaveRaoPlugin">
 *      <uri>ws://127.0.0.1:12345</uri>
 * </plugin>
 */

class WaveRaoPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate {
public:
    WaveRaoPlugin();
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:
    void createConnection();

private:
    // Json of t,position3d and orientation in quaternion
    nlohmann::json m_last_update_state_json;
    std::string m_uri;
    ConnectionType m_connection_type;
    std::string m_vessel_name;
    gz::sim::Entity m_entity;
    // std::shared_ptr<WaveRaoInterface> m_client;
    std::shared_ptr<XdynWebsocket> m_client;
};

} // namespace gazebo
} // namespace liquidai
#endif