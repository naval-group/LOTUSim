#ifndef __RENDER_PLUGIN_HH__
#define __RENDER_PLUGIN_HH__

#include "world_plugins/RenderInterface.h"
#include "world_plugins/UDPRenderInterface.h"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>

#include "gz/sim/Util.hh"
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>

namespace liquidai {
namespace gazebo {

std::shared_ptr<RenderInterfaceBase>
CreateRenderInterface(const std::string &protocol)
{
    if (protocol == "udp") {
        return std::make_shared<UDPRenderInterface>();
    }
    return nullptr;
};

class RenderPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
public:
    RenderPlugin();

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

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
     * @brief Vessel model entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessel_entity;

    std::shared_ptr<gz::transport::Node> m_gz_node;
    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    std::shared_ptr<RenderInterfaceBase> m_render_interface;
};

} // namespace gazebo
} // namespace liquidai
#endif