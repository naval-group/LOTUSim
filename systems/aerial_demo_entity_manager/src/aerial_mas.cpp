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

#include <gz/sim/components/Model.hh>

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
        m_aerial_namespace = "aerialWorld";
        m_logger->warn(
            "AerialMAS::customUserConfiguration: no aerial namespace given, "
            "defaulting to '{}'.",
            m_aerial_namespace);
    }

    // An invalid name makes create_client throw, which aborts gz-server in
    // Configure. Leave the client null instead; the hooks then skip mirroring.
    if (!lotusim::common::isValidRosName(m_aerial_namespace)) {
        m_logger->error(
            "AerialMAS::customUserConfiguration: invalid aerial namespace "
            "'{}', vessels will not be mirrored.",
            m_aerial_namespace);
        return;
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

bool AerialMAS::mirrorsFromAerialWorld(const gz::sim::Entity& entity) const
{
    // The agent SDK emits, per domain, a <physics_engine_interface> block. Only
    // the aerial one wired as <connection_type>ROS2</connection_type> means
    // "the host mirrors my pose from the aerial world", which is the single
    // case that needs a twin entity over there. A Kinematic aerial agent is
    // integrated locally and needs no twin; a surface or underwater agent needs
    // none either, and forwarding one is harmful: the aerial world then loads a
    // hull collision mesh its physics engine cannot handle.
    //
    // The SDF is read back from the spawned entity rather than the request, so
    // a <lotus_param> declared in the model's own model.sdf is seen too. The
    // lookup mirrors PhysicsInterfacePlugin::loadVessel and
    // createDomainInterface, so both agree on which agents are aerial ROS2.
    const auto* model_sdf =
        m_ecm->Component<gz::sim::components::ModelSdf>(entity);
    if (!model_sdf) {
        return false;
    }
    sdf::ElementPtr model_elem = model_sdf->Data().Element();
    if (!model_elem) {
        return false;
    }
    sdf::ElementPtr lotus_elem = model_elem->GetIncludeElement();
    if (!lotus_elem) {
        lotus_elem = model_elem;
    }
    if (!lotus_elem->HasElement("lotus_param") ||
        !lotus_elem->GetElement("lotus_param")
             ->HasElement("physics_engine_interface")) {
        return false;
    }
    sdf::ElementPtr aerial_sdf = lotusim::common::getElementCaseInsensitive(
        lotus_elem->GetElement("lotus_param")
            ->GetElement("physics_engine_interface"),
        "aerial");
    if (!aerial_sdf) {
        return false;
    }

    // connection_type is the legacy spelling and wins when both are present.
    std::string interface_type;
    if (aerial_sdf->HasElement("connection_type")) {
        interface_type = aerial_sdf->Get<std::string>("connection_type");
    } else if (aerial_sdf->HasElement("interface_type")) {
        interface_type = aerial_sdf->Get<std::string>("interface_type");
    }
    return lotusim::common::toUpper(interface_type) == "ROS2";
}

void AerialMAS::customUserAddEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    if (!m_aerial_mas_client) {
        return;
    }
    if (!mirrorsFromAerialWorld(msg.entity)) {
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
    if (!m_aerial_mas_client) {
        return;
    }
    // Only vessels we mirrored exist over there; deleting any other name would
    // be a goal the aerial world can only reject.
    if (m_forwarded_vessels.erase(msg.vessel_name) == 0) {
        return;
    }

    auto aerial_goal_msg = lotusim_msgs::action::MASCmd::Goal();
    aerial_goal_msg.header.stamp = rclcpp::Clock().now();
    aerial_goal_msg.header.frame_id = "world";
    aerial_goal_msg.cmd = msg;
    // When entity is set to 0, deleteEntity use vessel name instead
    aerial_goal_msg.cmd.entity = 0;

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