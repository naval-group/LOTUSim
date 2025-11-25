/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "render_interface/ros_interface.hpp"

namespace lotusim::gazebo {

// TODO: create explosion pipeline. Create default function update for special
// commands

ROSInterface::ROSInterface(
    const std::string& world_name,
    std::shared_ptr<spdlog::logger> logger)
    : RenderInterfaceBase(world_name, std::move(logger))
{
    m_ros_node = rclcpp::Node::make_shared("render_interface", m_world_name);
    m_logger = logger::createConsoleAndFileLogger(
        "ros2_render_interface",
        "ros2_render_interface.txt");
}

ROSInterface::~ROSInterface() {}

bool ROSInterface::configureInterface(
    const std::shared_ptr<const sdf::Element>&)
{
    m_logger->info(
        "ROSInterface::configureInterface : ROS renderer interface created");

    rclcpp::QoS qos_profile(10);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    m_renderer_cmd_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::RendererCmd>(
            "renderer_cmd", qos_profile);

    m_pose_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::VesselPositionArray>(
            "renderer_poses", qos_profile);
    return true;
}

bool ROSInterface::sendPosition(
    const std::chrono::steady_clock::duration& runTime,
    const std::vector<std::pair<std::string, gz::math::Pose3d>>& poses)
{
    lotusim_msgs::msg::VesselPositionArray array_msg;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(runTime).count();
    array_msg.header.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    array_msg.header.stamp.nanosec =
        static_cast<uint32_t>(simTimeNs % 1000000000);
    array_msg.header.frame_id = "world";

    for (const auto& pair : poses) {
        lotusim_msgs::msg::VesselPosition msg;
        msg.vessel_name = pair.first;

        const gz::math::Pose3d& pose = pair.second;
        msg.pose.position.x = pose.X();
        msg.pose.position.y = pose.Y();
        msg.pose.position.z = pose.Z();
        msg.pose.orientation.w = pose.Rot().W();
        msg.pose.orientation.x = pose.Rot().X();
        msg.pose.orientation.y = pose.Rot().Y();
        msg.pose.orientation.z = pose.Rot().Z();

        array_msg.vessels.push_back(msg);
    }
    m_pose_pub->publish(array_msg);
    return true;
}

/**
 * @brief Create subscriber for thruster command and update Unity to render
 * new vessel
 *
 * @param vessel_name
 * @param type_name
 * @return true
 * @return false
 */
bool ROSInterface::createVessel(
    const std::string& vessel_name,
    const gz::math::Pose3d& pose,
    sdf::ElementPtr sdfptr)
{
    std::string renderer_type_name = "";
    if (sdfptr->HasElement("renderer_type_name")) {
        renderer_type_name = sdfptr->Get<std::string>("renderer_type_name");
    }
    m_logger->info("RenderPlugin checking model: {}", vessel_name);

    // If renderer fails to spawn vessel, no point adding model to list to
    // update
    sendCreateMessage(vessel_name, pose, renderer_type_name);
    return true;
}

bool ROSInterface::destroyVessel(const std::string& vessel_name)
{
    sendDestroyMessage(vessel_name);
    // Always true as we will still remove models from updating list
    return true;
}
bool ROSInterface::customPreUpdates(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    return true;
}

bool ROSInterface::customUpdates(
    const gz::sim::UpdateInfo&,
    const gz::sim::EntityComponentManager&)
{
    return true;
}

void ROSInterface::sendCreateMessage(
    const std::string& name,
    const gz::math::Pose3d& pose,
    const std::string& type)
{
    lotusim_msgs::msg::RendererCmd render_msg;
    render_msg.cmd_type = lotusim_msgs::msg::RendererCmd::CREATE_CMD;
    render_msg.renderer_obj_name = type;
    render_msg.vessel_name = name;

    geometry_msgs::msg::Pose p;
    p.position.x = pose.X();
    p.position.y = pose.Y();
    p.position.z = pose.Z();
    p.orientation.x = pose.Rot().X();
    p.orientation.y = pose.Rot().Y();
    p.orientation.z = pose.Rot().Z();
    p.orientation.w = pose.Rot().W();

    render_msg.vessel_position = p;
    m_renderer_cmd_pub->publish(render_msg);
}

void ROSInterface::sendDestroyMessage(const std::string& name)
{
    lotusim_msgs::msg::RendererCmd render_msg;
    render_msg.header.stamp = m_ros_node->now();
    render_msg.cmd_type = lotusim_msgs::msg::RendererCmd::DELETE_CMD;
    render_msg.vessel_name = name;
    m_renderer_cmd_pub->publish(render_msg);
}

void ROSInterface::sendExplodeMessage(const std::string& name)
{
    lotusim_msgs::msg::RendererCmd render_msg;
    render_msg.header.stamp = m_ros_node->now();
    render_msg.cmd_type = lotusim_msgs::msg::RendererCmd::EXPLODE_CMD;
    render_msg.vessel_name = name;
    m_renderer_cmd_pub->publish(render_msg);
}

}  // namespace lotusim::gazebo