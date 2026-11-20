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

namespace {

geographic_msgs::msg::GeoPose toGeoPoseMsg(const GeoPose& pose)
{
    geographic_msgs::msg::GeoPose msg;
    msg.position.latitude = pose.position.latitude;
    msg.position.longitude = pose.position.longitude;
    msg.position.altitude = pose.position.altitude;
    msg.orientation.w = pose.orientation.W();
    msg.orientation.x = pose.orientation.X();
    msg.orientation.y = pose.orientation.Y();
    msg.orientation.z = pose.orientation.Z();
    return msg;
}

}  // namespace

ROSInterface::ROSInterface(
    const std::string& world_name,
    std::shared_ptr<spdlog::logger> logger,
    std::shared_ptr<rclcpp::Node> diag_node)
    : RenderInterfaceBase(world_name, std::move(logger), std::move(diag_node))
{
    m_ros_node = rclcpp::Node::make_shared("render_interface", m_world_name);
    const std::string logger_name =
        logger::worldLoggerName(m_world_name, "ros2_render_interface");
    m_logger = logger::createConsoleAndFileLogger(
        logger_name,
        logger_name + ".txt");
    logger::enableDiagnostics(m_logger, m_diag_node, m_world_name);
}

ROSInterface::~ROSInterface()
{
    if (m_logger) {
        m_logger->info(
            "ROSInterface::~ROSInterface: ROSInterface successfully shutdown.");
        logger::disableDiagnostics(m_logger);
    }
}

bool ROSInterface::configureInterface(
    const std::shared_ptr<const sdf::Element>&)
{
    m_logger->info(
        "ROSInterface::configureInterface : ROS renderer interface created");

    // renderer_state is a latched snapshot of the vessels alive right now and
    // their poses, sent by sendWorldState. A late-joining renderer receives
    // the current world in one message whose size depends only on the
    // current number of vessels.
    rclcpp::QoS renderer_state_qos_profile{rclcpp::KeepLast(1)};
    renderer_state_qos_profile.reliable();
    renderer_state_qos_profile.transient_local();

    m_renderer_state_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::RendererState>(
            "renderer_state",
            renderer_state_qos_profile);

    // renderer_cmd is a plain event stream for one-off events (EXPLODE).
    rclcpp::QoS renderer_cmd_qos_profile{rclcpp::KeepLast(10)};
    renderer_cmd_qos_profile.reliable();
    renderer_cmd_qos_profile.durability_volatile();

    m_renderer_cmd_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::RendererCmd>(
            "renderer_cmd",
            renderer_cmd_qos_profile);

    return true;
}

bool ROSInterface::sendWorldState(
    const std::chrono::steady_clock::duration& runTime,
    const std::vector<std::pair<std::string, GeoPose>>& poses)
{
    lotusim_msgs::msg::RendererState state_msg;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(runTime).count();
    state_msg.header.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    state_msg.header.stamp.nanosec =
        static_cast<uint32_t>(simTimeNs % 1000000000);
    state_msg.header.frame_id = "world";

    {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        state_msg.vessels.reserve(poses.size());
        for (const auto& [name, pose] : poses) {
            const auto it = m_live_vessels.find(name);
            if (it == m_live_vessels.end()) {
                continue;
            }
            lotusim_msgs::msg::RendererVessel vessel = it->second;
            vessel.vessel_pose = toGeoPoseMsg(pose);
            state_msg.vessels.push_back(std::move(vessel));
        }
    }
    m_renderer_state_pub->publish(state_msg);
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
    const GeoPose& pose,
    sdf::ElementPtr sdfptr)
{
    std::string renderer_type_name = "";
    if (sdfptr->HasElement("renderer_type_name")) {
        renderer_type_name = sdfptr->Get<std::string>("renderer_type_name");
    }
    m_logger->info("RenderPlugin checking model: {}", vessel_name);

    lotusim_msgs::msg::RendererVessel vessel;
    vessel.vessel_name = vessel_name;
    vessel.renderer_obj_name = renderer_type_name;
    vessel.vessel_pose = toGeoPoseMsg(pose);

    // Only record the vessel; it appears in the next renderer_state snapshot
    // sent by sendWorldState.
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_live_vessels[vessel_name] = std::move(vessel);
    return true;
}

bool ROSInterface::destroyVessel(const std::string& vessel_name)
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_live_vessels.erase(vessel_name);
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

void ROSInterface::sendExplodeMessage(const std::string& name)
{
    lotusim_msgs::msg::RendererCmd render_msg;
    render_msg.header.stamp = m_ros_node->now();
    render_msg.cmd_type = lotusim_msgs::msg::RendererCmd::EXPLODE_CMD;
    render_msg.vessel_name = name;
    m_renderer_cmd_pub->publish(render_msg);
}

}  // namespace lotusim::gazebo