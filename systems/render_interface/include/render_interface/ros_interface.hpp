/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_ROS_RENDER_INTERFACE_HPP_
#define LOTUSIM_ROS_RENDER_INTERFACE_HPP_

#include <gz/msgs/contacts.pb.h>

#include <boost/asio.hpp>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>

#include "gz/sim/Util.hh"
#include "lotusim_msgs/msg/renderer_cmd.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"
#include "render_interface/render_interface.hpp"

namespace lotusim::gazebo {

/**
 * @brief ROS interface for Renderer
 *
 */

class ROSInterface : public RenderInterfaceBase {
public:
    ROSInterface(
        const std::string& world_name,
        std::shared_ptr<spdlog::logger> logger);
    ~ROSInterface();

    bool configureInterface(
        const std::shared_ptr<const sdf::Element>& _sdf) override final;

    bool sendPosition(
        const std::chrono::steady_clock::duration& runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>>& poses)
        override final;

    /**
     * @brief Method to create new vessel
     * Handles reading of vessel params, and how the renderer create the vessel
     *
     * @param vessel_name
     * @param type_name
     * @return true
     * @return false
     */
    bool createVessel(
        const std::string& vessel_name,
        const gz::math::Pose3d& pose,
        sdf::ElementPtr sdfptr) override final;

    bool destroyVessel(const std::string& vessel_name) override final;

    virtual bool customPreUpdates(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override final;

    virtual bool customUpdates(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override final;

private:
    void sendCreateMessage(
        const std::string& name,
        const gz::math::Pose3d& pose,
        const std::string& type);

    void sendDestroyMessage(const std::string& name);

    void sendExplodeMessage(const std::string& name);

private:
    rclcpp::Node::SharedPtr m_ros_node;

    rclcpp::Publisher<lotusim_msgs::msg::RendererCmd>::SharedPtr
        m_renderer_cmd_pub;
    rclcpp::Publisher<lotusim_msgs::msg::VesselPositionArray>::SharedPtr
        m_pose_pub;
};

}  // namespace lotusim::gazebo
#endif