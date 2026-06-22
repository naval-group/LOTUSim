/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/default_platform_power_manager.hpp"
#include "power_subsystem/platform_power_manager_base.hpp"

namespace lotusim::gazebo {

std::unique_ptr<PlatformPowerManagerBase> PlatformPowerManagerBase::create(
    PlatformPowerManagerType type,
    gz::sim::Entity vessel_entity,
    const std::string& vessel_name,
    rclcpp::Node::SharedPtr node,
    sdf::ElementPtr sdfptr)
{
    switch (type) {
        case PlatformPowerManagerType::DEFAULT:
        default:
            return std::make_unique<DefaultPlatformPowerManager>(
                vessel_entity,
                vessel_name,
                std::move(node),
                sdfptr);
    }
}

}  // namespace lotusim::gazebo
