/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_OCEAN_CURRENT_FEED_HH_
#define LOTUSIM_OCEAN_CURRENT_FEED_HH_

#include <lotusim_msgs/msg/ocean_current.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lotusim::gazebo {

/**
 * @brief Fake, uniform ocean current feed for KinematicInterface.
 *
 * NOT a general physics feature, and deliberately kept out of
 * PhysicsInterfacePlugin/kinematic_interface.cpp's own files: it only
 * affects entities on the Kinematic connection type (a manual/demo
 * fallback with no real hydrodynamics to begin with). A vessel that wants a
 * physically-simulated current should use XDynWebSocket instead, whose own
 * hydrodynamic config already carries a real "ekman current" model — this
 * class is unrelated to that path and never touches it. Living in its own
 * translation unit means a deployment that doesn't want this fake-current
 * demo feature can simply not construct one (or drop this file from the
 * build) without touching any of the shared plugin code.
 *
 * Subscribes to `<node namespace>/ocean_current` (lotusim_msgs/OceanCurrent,
 * world-frame ENU m/s, TRANSIENT_LOCAL so the OceanCurrent SDK agent's
 * latched publishes are delivered regardless of subscribe/publish ordering)
 * and forwards every message into KinematicInterface::setCurrent(). The
 * ``enable_current`` flag is how the agent signals shutdown: it publishes a
 * default-constructed (disabled, zero-vector) message from its
 * ``destroy_node()``, which this feed treats as "reset to no current"
 * instead of a genuine (0, 0) current value. ``linear_velocity.z`` is read
 * but unused: KinematicInterface's pose integration is 2D (x/y + yaw) only.
 */
class OceanCurrentFeed {
public:
    explicit OceanCurrentFeed(const rclcpp::Node::SharedPtr& node);

private:
    rclcpp::Subscription<lotusim_msgs::msg::OceanCurrent>::SharedPtr m_sub;
};

}  // namespace lotusim::gazebo

#endif
