#ifndef __RENDER_PLUGIN_HH__
#define __RENDER_PLUGIN_HH__

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>

#include "RenderInterface.h"
#include "TcpUdpInterface.h"
#include "gz/sim/Util.hh"

namespace lotusim::gazebo {

std::shared_ptr<RenderInterfaceBase> CreateRenderInterface(
    const std::string &protocol,
    std::shared_ptr<spdlog::logger> logger)
{
    if (protocol == "TCPUDP") {
        return std::make_shared<TcpUdpInterface>(logger);
    }
    return nullptr;
};

/**
 * @brief RenderPlugin is for basic rendering functions.
 * For more specific functions, please create your own user interface function
 *
 * <plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">
 *      <connection_protocol>TCPUDP</connection_protocol>
 *      <ip>127.0.0.1</ip>
 *      <udp_port>23456</udp_port>
 *      <tcp_port>23457</tcp_port>
 * </plugin>
 *
 * PLease add the tags to the models
 * <renderer_type_name>
 * <publish_render>
 */
class RenderPlugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate {
public:
    RenderPlugin();

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) final;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief Vessel model entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessel_entity;

    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    std::shared_ptr<RenderInterfaceBase> m_render_interface;
};

}  // namespace lotusim::gazebo
#endif