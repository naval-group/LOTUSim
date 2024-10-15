#ifndef __AIS_PLUGIN_HH__
#define __AIS_PLUGIN_HH__

#include <string.h>

#include <chrono>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz_liquidai_msgs/msgs/aismsg.pb.h"
#include "logging_system/logger.hpp"

namespace lotusim::gazebo {

/**
 * @brief This is a plugin for AIS array publisher.
 * All vessel in the world should contains the tag <publish_ais> in sdf.
 *
 * <plugin filename="ais_plugin" name="lotusim::gazebo::AISPlugin">
 *      <period> 1 </period>
 * </plugin>
 *
 * <publish_ais>false</publish_ais>
 */
class AISPlugin : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPostUpdate {
public:
    AISPlugin();

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) final;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    gz::transport::Node::Publisher m_ais_pub;
    std::shared_ptr<gz::transport::Node> m_gz_node;
    std::vector<gz::sim::Entity> m_vessel_entities;

    std::vector<gz::sim::Entity> m_vessel_entities_pose;
    std::vector<std::pair<std::string, gz::math::Pose3d>> m_pose;
};

}  // namespace lotusim::gazebo
#endif