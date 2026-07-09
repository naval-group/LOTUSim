/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_LIGHT_ACTUATOR_PLUGIN_HPP_
#define LOTUSIM_LIGHT_ACTUATOR_PLUGIN_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "lotusim_common/logger.hpp"

namespace lotusim::actuator {

/// \brief A model-attached actuator that turns a status "LED" on/off from a
/// ROS 2 std_msgs/Bool topic by toggling the emissive colour of a model
/// <visual>.
///
/// Why a visual and not a <light>: runtime changes to a <light>'s intensity
/// (LightCmd) are not reliably reflected in the gz-sim GUI render (the server
/// render engine and the GUI render engine are separate). Toggling a visual's
/// emissive material via VisualCmd is the same mechanism the visual_config
/// service uses and shows up in the GUI — and an emissive colour glows on its
/// own, independent of scene lighting/ambient.
///
/// Mirrors the way sensors publish ROS directly: the plugin runs inside the
/// Gazebo server (co-located with the simulation), owns its own ROS node, so a
/// remote ROS-only client only has to publish a Bool — no gz transport needed.
///
/// Declared inside a <model>:
/// ```
/// <plugin filename="light_actuator"
///         name="lotusim::actuator::LightActuatorPlugin">
///   <visual_name>led_body_visual</visual_name>  <!-- visual to drive (required) -->
/// </plugin>
/// ```
/// Subscribes to ``/<world>/<model>/light/cmd`` (std_msgs/Bool): true = ON
/// (emissive red), false = OFF (emissive black).
class LightActuatorPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
public:
    LightActuatorPlugin();
    ~LightActuatorPlugin() override;

    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    void applyEmissive(gz::sim::EntityComponentManager& _ecm, bool _on);

    std::shared_ptr<spdlog::logger> m_logger;

    // Gazebo
    gz::sim::Entity m_model_entity{gz::sim::kNullEntity};
    gz::sim::Entity m_visual_entity{gz::sim::kNullEntity};
    std::string m_model_name;
    std::string m_visual_name{"led_body_visual"};
    std::string m_world_name;

    // Latest command from ROS (written by the executor thread, read by PreUpdate)
    std::mutex m_cmd_mutex;
    bool m_desired_on{false};
    bool m_cmd_dirty{false};

    // ROS 2
    rclcpp::Node::SharedPtr m_ros_node;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_cmd_sub;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::shared_ptr<std::thread> m_ros_thread;
};

}  // namespace lotusim::actuator

#endif  // LOTUSIM_LIGHT_ACTUATOR_PLUGIN_HPP_
