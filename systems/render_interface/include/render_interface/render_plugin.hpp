/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_RENDER_PLUGIN_HPP_
#define LOTUSIM_RENDER_PLUGIN_HPP_

#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <mutex>

#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "render_interface/render_interface.hpp"
#include "render_interface/ros_interface.hpp"
#include "render_interface/tcpudp_interface.hpp"

namespace lotusim::gazebo {

std::shared_ptr<RenderInterfaceBase> CreateRenderInterface(
    const std::string& protocol,
    const std::string& world_name,
    std::shared_ptr<spdlog::logger> logger)
{
    if (protocol == "TCPUDP") {
        return std::make_shared<TcpUdpInterface>(world_name, logger);
    }
    if (protocol == "ROS2") {
        return std::make_shared<ROSInterface>(world_name, logger);
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

    ~RenderPlugin();

    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief World name
     *
     */
    std::string m_world_name;

    std::mutex m_mutex;

    /**
     * @brief Mapping of vessel name to model entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessel_entity;

    /**
     * @brief Render interface implementation
     *
     */
    std::shared_ptr<RenderInterfaceBase> m_render_interface;
};

}  // namespace lotusim::gazebo
#endif