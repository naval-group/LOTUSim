/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef AERIAL_mas_HPP_
#define AERIAL_mas_HPP_

#include <set>
#include <string>

#include "lotusim_mas/multi_agent_system.hpp"

namespace lotusim::gazebo {

using ClientGoalHandleMASCmd =
    rclcpp_action::ClientGoalHandle<lotusim_msgs::action::MASCmd>;

class AerialMAS : public MultiAgentSystem {
public:
    AerialMAS();
    ~AerialMAS();

private:
    void customUserConfiguration(
        const std::shared_ptr<const sdf::Element>& _sdf) override;

    void customUserAddEntity(
        const lotusim_msgs::msg::MASCmd& msg) override final;

    void customUserDeleteEntity(
        const lotusim_msgs::msg::MASCmd& msg) override final;

    /// True when the entity being created asks the host to mirror it from the
    /// aerial world, i.e. its SDF declares an <aerial> domain wired over ROS2.
    /// Every other entity belongs to this world alone and must not be forwarded.
    static bool mirrorsFromAerialWorld(const lotusim_msgs::msg::MASCmd& msg);

    void resultCB(const ClientGoalHandleMASCmd::WrappedResult& result);

    void goalResponseCB(ClientGoalHandleMASCmd::SharedPtr goal_handle);

private:
    rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SharedPtr
        m_aerial_mas_client;

    std::string m_aerial_namespace;

    /// Vessels actually forwarded on creation. A DELETE_CMD carries no SDF, so
    /// it cannot be classified on its own: only a vessel we mirrored may be
    /// deleted in the aerial world.
    std::set<std::string> m_forwarded_vessels;
};
}  // namespace lotusim::gazebo
#endif