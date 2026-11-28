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

/// \brief Turns a status LED on and off from a ROS 2 std_msgs/Bool topic by
/// toggling the emissive colour of a model <visual>.
///
/// A visual is used rather than a <light> because emissive changes show up in
/// the GUI render and glow independently of scene lighting.
///
/// Declared inside a <model>:
/// ```
/// <plugin filename="light_actuator"
///         name="lotusim::actuator::LightActuatorPlugin">
///   <visual_name>led_body_visual</visual_name>
/// </plugin>
/// ```
/// Subscribes to ``/<world>/<model>/light/cmd``: true is on, false is off.
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
