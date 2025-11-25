/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef AERIAL_ENTITY_MANAGER_HPP_
#define AERIAL_ENTITY_MANAGER_HPP_

#include "entity_manager/entity_manager.hpp"

namespace lotusim::gazebo {

using ClientGoalHandleMASCmd =
    rclcpp_action::ClientGoalHandle<lotusim_msgs::action::MASCmd>;

class AerialEntityManager : public EntityManager {
public:
    AerialEntityManager();
    ~AerialEntityManager();

private:
    void customUserConfiguration(
        const std::shared_ptr<const sdf::Element> &_sdf);

    void customUserAddEntity(
        const lotusim_msgs::msg::MASCmd &msg) override final;

    void customUserDeleteEntity(
        const lotusim_msgs::msg::MASCmd &msg) override final;

    void resultCB(const ClientGoalHandleMASCmd::WrappedResult &result);

    void goalResponseCB(ClientGoalHandleMASCmd::SharedPtr goal_handle);

private:
    rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SharedPtr
        m_aerial_entity_manager_client;

    std::string m_aerial_namespace;
};
}  // namespace lotusim::gazebo
#endif