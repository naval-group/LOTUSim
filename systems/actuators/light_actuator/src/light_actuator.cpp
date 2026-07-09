/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "light_actuator/light_actuator.hpp"

#include <gz/plugin/Register.hh>

#include <gz/msgs/visual.pb.h>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>

#include "lotusim_common/common.hpp"

namespace lotusim::actuator {

LightActuatorPlugin::LightActuatorPlugin()
{
    m_logger = logger::createConsoleAndFileLogger(
        "light_actuator",
        "light_actuator.txt");
}

LightActuatorPlugin::~LightActuatorPlugin()
{
    // Stop only our own executor/thread — do NOT call rclcpp::shutdown() here,
    // it would tear down the ROS context shared with the other plugins/sensors
    // in this same Gazebo process.
    if (m_executor)
        m_executor->cancel();
    if (m_ros_thread && m_ros_thread->joinable())
        m_ros_thread->join();
}

void LightActuatorPlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/)
{
    m_model_entity = _entity;
    m_world_name = lotusim::common::getWorldName(_ecm);

    auto name_opt = _ecm.Component<gz::sim::components::Name>(_entity);
    if (!name_opt) {
        m_logger->error(
            "LightActuatorPlugin::Configure: model entity has no Name "
            "component; cannot set up ROS topic.");
        return;
    }
    m_model_name = name_opt->Data();

    auto sdfPtr = const_cast<sdf::Element*>(_sdf.get());
    if (sdfPtr->HasElement("visual_name"))
        m_visual_name = sdfPtr->Get<std::string>("visual_name");

    // Own ROS node, namespaced on the world (same convention as the sensors and
    // the waypoint follower → topic resolves to /<world>/<model>/light/cmd).
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

    m_ros_node = rclcpp::Node::make_shared("light_actuator", m_world_name);
    m_cmd_sub = m_ros_node->create_subscription<std_msgs::msg::Bool>(
        m_model_name + "/light/cmd",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            m_desired_on = msg->data;
            m_cmd_dirty = true;
        });

    m_executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_ros_node);
    m_ros_thread =
        std::make_shared<std::thread>([this]() { m_executor->spin(); });

    m_logger->info(
        "LightActuatorPlugin::Configure: model '{}' driving visual '{}', "
        "listening on /{}/{}/light/cmd",
        m_model_name,
        m_visual_name,
        m_world_name,
        m_model_name);
}

void LightActuatorPlugin::applyEmissive(
    gz::sim::EntityComponentManager& _ecm, bool _on)
{
    // Command the visual's material emissive colour. Emissive glows on its own
    // (no lighting needed) so ON = bright red, OFF = black (back to the base
    // grey material). VisualCmd is the runtime visual-update channel that the
    // GUI render honours.
    gz::msgs::Visual visualMsg;
    visualMsg.set_id(static_cast<uint32_t>(m_visual_entity));
    auto* material = visualMsg.mutable_material();
    auto* emissive = material->mutable_emissive();
    emissive->set_r(_on ? 1.0f : 0.0f);
    emissive->set_g(0.0f);
    emissive->set_b(0.0f);
    emissive->set_a(1.0f);

    _ecm.SetComponentData<gz::sim::components::VisualCmd>(
        m_visual_entity, visualMsg);
    _ecm.SetChanged(
        m_visual_entity,
        gz::sim::components::VisualCmd::typeId,
        gz::sim::ComponentState::OneTimeChange);
}

void LightActuatorPlugin::PreUpdate(
    const gz::sim::UpdateInfo& /*_info*/,
    gz::sim::EntityComponentManager& _ecm)
{
    // Lazily resolve the visual entity once it exists: the visual whose Name
    // matches and whose top-level model is THIS model (multiple agents share the
    // same SDF, so the visual name alone is not unique across instances).
    if (m_visual_entity == gz::sim::kNullEntity) {
        _ecm.Each<gz::sim::components::Visual, gz::sim::components::Name>(
            [&](const gz::sim::Entity& _e,
                const gz::sim::components::Visual*,
                const gz::sim::components::Name* _name) -> bool {
                if (_name->Data() == m_visual_name &&
                    gz::sim::topLevelModel(_e, _ecm) == m_model_entity) {
                    m_visual_entity = _e;
                    return false;  // stop iterating
                }
                return true;
            });
        if (m_visual_entity == gz::sim::kNullEntity)
            return;  // visual not spawned yet, try again next tick
    }

    // Apply the latest command, if any.
    bool on;
    {
        std::lock_guard<std::mutex> lock(m_cmd_mutex);
        if (!m_cmd_dirty)
            return;
        on = m_desired_on;
        m_cmd_dirty = false;
    }

    applyEmissive(_ecm, on);
    m_logger->info(
        "LightActuatorPlugin: '{}' visual '{}' turned {}",
        m_model_name,
        m_visual_name,
        on ? "ON" : "OFF");
}

}  // namespace lotusim::actuator

GZ_ADD_PLUGIN(
    lotusim::actuator::LightActuatorPlugin,
    gz::sim::System,
    lotusim::actuator::LightActuatorPlugin::ISystemConfigure,
    lotusim::actuator::LightActuatorPlugin::ISystemPreUpdate)
