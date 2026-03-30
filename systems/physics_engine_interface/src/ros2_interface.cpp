/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/ros2_interface.hpp"

namespace lotusim::gazebo {

std::shared_ptr<ROS2Interface> ROS2Interface::m_instance = nullptr;

std::shared_ptr<ROS2Interface> ROS2Interface::createInterface()
{
    if (m_instance == nullptr) {
        m_instance = std::make_shared<ROS2Interface>();
    }
    return m_instance;
}

ROS2Interface::ROS2Interface() : PhysicsInterfaceBase("Ros2Interface")
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    m_ros_node = rclcpp::Node::make_shared("physics_aerial_linker");
    if (rclcpp::ok()) {
        m_executor =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_ros_node);
        m_ros_node_thread =
            std::make_shared<std::thread>([&]() { m_executor->spin(); });
    }
}

bool ROS2Interface::configureInterface(
    const gz::sim::Entity& _entity,
    const std::string& _name,
    const sdf::ElementPtr _sdf,
    const DomainType& /*domain_type*/)
{
    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);

    std::string _namespace;
    if (_sdf->HasElement("namespace")) {
        _namespace = _sdf->Get<std::string>("namespace");
    } else {
        _namespace = "";
    }

    if (m_namespace.find(_namespace) == m_namespace.end()) {
        auto pose_sub =
            m_ros_node
                ->create_subscription<lotusim_msgs::msg::VesselPositionArray>(
                    _namespace + "/poses",
                    1,
                    std::bind(
                        &ROS2Interface::aerialPosesCB,
                        this,
                        std::placeholders::_1));
        m_pose_sub.push_back(pose_sub);
    }
    m_namespace[_namespace].insert(_entity);
    m_entity_name_map[_entity] = _name;
    return true;
}

bool ROS2Interface::removeInterface(
    const gz::sim::Entity& _entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
    m_entity_name_map.erase(_entity);
    return true;
}

void ROS2Interface::aerialPosesCB(
    const lotusim_msgs::msg::VesselPositionArray::ConstSharedPtr& msg)
{
    for (auto& vessel : msg->vessels) {
        m_vessel_pose_map[vessel.vessel_name] = vessel.pose;
    }
}

std::optional<std::tuple<VesselInformation, DomainType>>
ROS2Interface::getNewState(
    const gz::sim::Entity& _entity,
    const VesselInformation& previous_state,
    float time_dif)
{
    std::shared_lock<std::shared_mutex> lock(m_variable_mutex);
    auto name_it = m_entity_name_map.find(_entity);
    if (name_it == m_entity_name_map.end()) {
        return std::nullopt;
    }

    auto pose_it = m_vessel_pose_map.find(name_it->second);
    if (pose_it == m_vessel_pose_map.end()) {
        return std::nullopt;
    }

    const geometry_msgs::msg::Pose& pose = pose_it->second;

    // Create VesselInformation from ROS2 pose message
    VesselInformation vessel_info;
    vessel_info.time = previous_state.time + time_dif;
    vessel_info.entity = _entity;

    // Convert geometry_msgs::Pose to gz::math::Pose3d
    vessel_info.pose.Pos().Set(
        pose.position.x,
        pose.position.y,
        pose.position.z);

    vessel_info.pose.Rot().Set(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    // Set velocities to zero, currently not supported
    vessel_info.lin_vel.Set(0.0, 0.0, 0.0);
    vessel_info.ang_vel.Set(0.0, 0.0, 0.0);

    return std::make_optional(std::make_tuple(vessel_info, DomainType::Aerial));
}

bool ROS2Interface::activateInterface(const gz::sim::Entity&, const DomainType&)
{
    return true;
}

bool ROS2Interface::deactivateInterface(
    const gz::sim::Entity&,
    const DomainType&)
{
    return true;
}

std::string ROS2Interface::getURI(const gz::sim::Entity&, const DomainType&)
{
    return "ros2";
}

}  // namespace lotusim::gazebo