/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "custom_aerial_mas/aerial_mas.hpp"

namespace lotusim::gazebo {

AerialMAS::AerialMAS() : MultiAgentSystem() {}

AerialMAS::~AerialMAS() {}

void AerialMAS::customUserConfiguration(
    const std::shared_ptr<const sdf::Element>& _sdf)
{
    if (_sdf->HasElement("aerial_namespace")) {
        m_aerial_namespace = _sdf->Get<std::string>("aerial_namespace");
        m_logger->info(
            "AerialMAS::customUserConfiguration: aerial manager namespace {}",
            m_aerial_namespace);
    } else {
        m_logger->error(
            "AerialMAS::customUserConfiguration: no aerial namespace given.");
    }

    m_aerial_mas_client =
        rclcpp_action::create_client<lotusim_msgs::action::MASCmd>(
            m_ros_node,
            "/" + m_aerial_namespace + "/mas_cmd");

    // Wait for aerial gz for 10 secs
    if (!m_aerial_mas_client->wait_for_action_server(
            std::chrono::seconds(10))) {
        m_logger->error(
            "AerialMAS::AerialMAS: Aerial gz world MAS not available");
    }
}

bool AerialMAS::mirrorsFromAerialWorld(const lotusim_msgs::msg::MASCmd& msg)
{
    // The agent SDK emits, per domain, a <physics_engine_interface> block. Only
    // the aerial one wired as <connection_type>ROS2</connection_type> means "the
    // host mirrors my pose from the aerial world", which is the single case that
    // needs a twin entity over there. A Kinematic aerial agent is integrated
    // locally and needs no twin; a surface or underwater agent needs none either,
    // and forwarding one is harmful: the aerial world then loads a hull
    // collision mesh its physics engine cannot handle.
    const std::string& sdf = msg.sdf_string;
    const auto open = sdf.find("<aerial>");
    if (open == std::string::npos) {
        return false;
    }
    const auto close = sdf.find("</aerial>", open);
    if (close == std::string::npos) {
        return false;
    }
    return sdf.find("ROS2", open) < close;
}

void AerialMAS::customUserAddEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    if (!mirrorsFromAerialWorld(msg)) {
        m_logger->info(
            "AerialMAS::customUserAddEntity: '{}' is not mirrored from the "
            "aerial world; not forwarding.",
            msg.vessel_name);
        return;
    }
    m_forwarded_vessels.insert(msg.vessel_name);

    auto aerial_goal_msg = lotusim_msgs::action::MASCmd::Goal();
    aerial_goal_msg.header.stamp = rclcpp::Clock().now();
    aerial_goal_msg.header.frame_id = "world";
    aerial_goal_msg.cmd = msg;

    auto send_goal_options =
        rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&AerialMAS::goalResponseCB, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&AerialMAS::resultCB, this, std::placeholders::_1);

    m_aerial_mas_client->async_send_goal(aerial_goal_msg, send_goal_options);
}

void AerialMAS::customUserDeleteEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    // Only vessels we mirrored exist over there; deleting any other name would
    // be a goal the aerial world can only reject.
    if (m_forwarded_vessels.erase(msg.vessel_name) == 0) {
        return;
    }

    auto aerial_goal_msg = lotusim_msgs::action::MASCmd::Goal();
    aerial_goal_msg.header.stamp = rclcpp::Clock().now();
    aerial_goal_msg.header.frame_id = "world";
    aerial_goal_msg.cmd = msg;

    auto send_goal_options =
        rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&AerialMAS::goalResponseCB, this, std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&AerialMAS::resultCB, this, std::placeholders::_1);

    m_aerial_mas_client->async_send_goal(aerial_goal_msg, send_goal_options);
}

void AerialMAS::goalResponseCB(ClientGoalHandleMASCmd::SharedPtr goal_handle)
{
    if (!goal_handle) {
        m_logger->error(
            "AerialMAS::goalResponseCB: MASCmd was rejected by server");
    } else {
        m_logger->info("AerialMAS::goalResponseCB: MASCmd accepted by server");
    }
}

void AerialMAS::resultCB(const ClientGoalHandleMASCmd::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            m_logger->info("AerialMAS::resultCB: MASCmd succeed");
            break;
        default:
            m_logger->info("AerialMAS::resultCB: MASCmd failed");
            break;
    }
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::AerialMAS,
    gz::sim::System,
    lotusim::gazebo::AerialMAS::ISystemConfigure,
    lotusim::gazebo::AerialMAS::ISystemPreUpdate,
    lotusim::gazebo::AerialMAS::ISystemUpdate,
    lotusim::gazebo::AerialMAS::ISystemPostUpdate)