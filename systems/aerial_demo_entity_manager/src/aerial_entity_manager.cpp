/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "custom_aerial_entity_manager/aerial_entity_manager.hpp"

namespace lotusim::gazebo {

AerialEntityManager::AerialEntityManager() : EntityManager() {}

AerialEntityManager::~AerialEntityManager() {}

void AerialEntityManager::customUserConfiguration(
    const std::shared_ptr<const sdf::Element>& _sdf)
{
    if (_sdf->HasElement("aerial_namespace")) {
        m_aerial_namespace = _sdf->Get<std::string>("aerial_namespace");
        m_logger->info(
            "AerialEntityManager::customUserConfiguration: aerial manager namespace {}",
            m_aerial_namespace);
    } else {
        m_logger->error(
            "AerialEntityManager::customUserConfiguration: no aerial namespace given.");
    }

    m_aerial_entity_manager_client =
        rclcpp_action::create_client<lotusim_msgs::action::MASCmd>(
            m_ros_node,
            "/" + m_aerial_namespace + "/mas_cmd");

    // Wait for aerial gz for 10 secs
    if (!m_aerial_entity_manager_client->wait_for_action_server(
            std::chrono::seconds(10))) {
        m_logger->error(
            "AerialEntityManager::AerialEntityManager: Aerial gz world MAS not available");
    }
}

void AerialEntityManager::customUserAddEntity(
    const lotusim_msgs::msg::MASCmd& msg)
{
    auto aerial_goal_msg = lotusim_msgs::action::MASCmd::Goal();
    aerial_goal_msg.header.stamp = rclcpp::Clock().now();
    aerial_goal_msg.header.frame_id = "world";
    aerial_goal_msg.cmd = msg;

    auto send_goal_options =
        rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &AerialEntityManager::goalResponseCB,
        this,
        std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&AerialEntityManager::resultCB, this, std::placeholders::_1);

    m_aerial_entity_manager_client->async_send_goal(
        aerial_goal_msg,
        send_goal_options);
}

void AerialEntityManager::customUserDeleteEntity(
    const lotusim_msgs::msg::MASCmd& msg)
{
    auto aerial_goal_msg = lotusim_msgs::action::MASCmd::Goal();
    aerial_goal_msg.header.stamp = rclcpp::Clock().now();
    aerial_goal_msg.header.frame_id = "world";
    aerial_goal_msg.cmd = msg;

    auto send_goal_options =
        rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &AerialEntityManager::goalResponseCB,
        this,
        std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&AerialEntityManager::resultCB, this, std::placeholders::_1);

    m_aerial_entity_manager_client->async_send_goal(
        aerial_goal_msg,
        send_goal_options);
}

void AerialEntityManager::goalResponseCB(
    ClientGoalHandleMASCmd::SharedPtr goal_handle)
{
    if (!goal_handle) {
        m_logger->error(
            "AerialEntityManager::goalResponseCB: MASCmd was rejected by server");
    } else {
        m_logger->info(
            "AerialEntityManager::goalResponseCB: MASCmd accepted by server");
    }
}

void AerialEntityManager::resultCB(
    const ClientGoalHandleMASCmd::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            m_logger->info("AerialEntityManager::resultCB: MASCmd succeed");
            break;
        default:
            m_logger->info("AerialEntityManager::resultCB: MASCmd failed");
            break;
    }
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::AerialEntityManager,
    gz::sim::System,
    lotusim::gazebo::AerialEntityManager::ISystemConfigure,
    lotusim::gazebo::AerialEntityManager::ISystemPreUpdate,
    lotusim::gazebo::AerialEntityManager::ISystemUpdate,
    lotusim::gazebo::AerialEntityManager::ISystemPostUpdate)