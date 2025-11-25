/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_ROS2_INTERFACE_HH_
#define LOTUSIM_ROS2_INTERFACE_HH_

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>
#include <unordered_map>

#include "lotusim_msgs/msg/vessel_position_array.hpp"
#include "physics_engine_interface/physics_interface_base.hpp"

namespace lotusim::gazebo {

/**
 * @brief This physics interface pulls pose data from ROS2 topic
 * "<namespace>/poses" and update pose of entity with similar name in the world
 *
 */
class ROS2Interface : public PhysicsInterfaceBase {
public:
    ROS2Interface(std::shared_ptr<const sdf::Element> _sdf);

    /**
     * @brief Static function to get static instance
     *
     * @param _sdf SDF of the plugin
     * @return std::shared_ptr<ROS2Interface>
     */
    static std::shared_ptr<ROS2Interface> getInstance(
        std::shared_ptr<const sdf::Element> _sdf);

    std::optional<std::tuple<VesselInformation, DomainType>> getNewState(
        const gz::sim::Entity& _entity,
        const VesselInformation& previous_state,
        float time_dif) override;

    bool createConnection(
        const gz::sim::Entity& _entity,
        const std::string& _name,
        const sdf::ElementPtr _sdf) override;

    bool removeConnection(const gz::sim::Entity& _entity) override;

    bool activateConnection(const gz::sim::Entity& _entity) override;

    bool deactivateConnection(const gz::sim::Entity& _entity) override;

    std::string getURI(const gz::sim::Entity& _entity) override;

private:
    void aerialPosesCB(
        const lotusim_msgs::msg::VesselPositionArray::ConstSharedPtr& msg);

private:
    static std::shared_ptr<ROS2Interface> m_instance;

    std::string m_namespace;
    /**
     * @brief ROS node
     *
     */
    rclcpp::Node::SharedPtr m_ros_node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::shared_ptr<std::thread> m_ros_node_thread;
    rclcpp::Subscription<lotusim_msgs::msg::VesselPositionArray>::SharedPtr
        m_pose_sub;

    mutable std::shared_mutex m_variable_mutex;

    /**
     * @brief Mapping of gazebo entity to name
     *
     */
    std::unordered_map<gz::sim::Entity, std::string> m_entity_name_map;

    /**
     * @brief Mapping of vessel name to pose
     *
     */
    std::unordered_map<std::string, geometry_msgs::msg::Pose> m_vessel_pose_map;
};

}  // namespace lotusim::gazebo

#endif