/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/ocean_current_feed.hpp"

#include "physics_engine_interface/kinematic_interface.hpp"

namespace lotusim::gazebo {

OceanCurrentFeed::OceanCurrentFeed(const rclcpp::Node::SharedPtr& node)
{
    m_sub = node->create_subscription<lotusim_msgs::msg::OceanCurrent>(
        "ocean_current",
        rclcpp::QoS(1).transient_local(),
        [node](lotusim_msgs::msg::OceanCurrent::ConstSharedPtr msg) -> void {
            const auto& v = msg->linear_velocity;
            KinematicInterface::getInstance()->setCurrent(
                msg->enable_current ? v.x : 0.0, msg->enable_current ? v.y : 0.0);
            // Kinematic entities only, by design — an xdyn vessel gets its
            // current from its own YAML (or the Gauss-Markov/Copernicus
            // injection point), where it is depth-resolved and physically
            // modelled, and applying a second uniform one here would
            // double-count it. Said once and out loud so that a mixed
            // scenario does not leave anyone wondering why its xdyn vessels
            // ignore the topic.
            RCLCPP_INFO_ONCE(
                node->get_logger(),
                "ocean_current applies to Kinematic entities only, aerial ones "
                "excepted; xdyn vessels take their current from their own YAML "
                "environment models or the Gauss-Markov/Copernicus injection "
                "point.");
        });
}

}  // namespace lotusim::gazebo
