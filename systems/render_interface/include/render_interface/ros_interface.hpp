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
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "gz/sim/Util.hh"
#include "lotusim_msgs/msg/renderer_cmd.hpp"
#include "lotusim_msgs/msg/renderer_state.hpp"
#include "lotusim_msgs/msg/renderer_vessel.hpp"
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
        std::shared_ptr<spdlog::logger> logger,
        std::shared_ptr<rclcpp::Node> diag_node);
    ~ROSInterface();

    bool configureInterface(
        const std::shared_ptr<const sdf::Element>& _sdf) override final;

    /**
     * @brief Publish the renderer_state snapshot: every live vessel with its
     * current geographic pose.
     */
    bool sendWorldState(
        const std::chrono::steady_clock::duration& runTime,
        const std::vector<std::pair<std::string, GeoPose>>& poses)
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
        const GeoPose& pose,
        sdf::ElementPtr sdfptr) override final;

    bool destroyVessel(const std::string& vessel_name) override final;

    virtual bool customPreUpdates(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override final;

    virtual bool customUpdates(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override final;

private:
    void sendExplodeMessage(const std::string& name);

private:
    rclcpp::Node::SharedPtr m_ros_node;

    /**
     * @brief Snapshot of every live vessel and its pose, published by
     * sendWorldState (KeepLast(1), TRANSIENT_LOCAL)
     */
    rclcpp::Publisher<lotusim_msgs::msg::RendererState>::SharedPtr
        m_renderer_state_pub;

    /**
     * @brief One-off renderer events such as EXPLODE (VOLATILE, KeepLast)
     */
    rclcpp::Publisher<lotusim_msgs::msg::RendererCmd>::SharedPtr
        m_renderer_cmd_pub;

    /**
     * @brief Vessels currently alive, keyed by name: the renderer object
     * name of each, filled into the renderer_state snapshot with its pose.
     */
    std::map<std::string, lotusim_msgs::msg::RendererVessel> m_live_vessels;

    std::mutex m_state_mutex;
};

}  // namespace lotusim::gazebo
#endif