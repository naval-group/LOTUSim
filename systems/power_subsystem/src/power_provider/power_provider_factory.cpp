/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_provider/power_provider.hpp"
#include "power_subsystem/power_provider/simple_battery.hpp"
#include "power_subsystem/power_provider/simple_generator.hpp"
#include "power_subsystem/power_provider/rpm_generator.hpp"

namespace lotusim::gazebo {

PowerProvider::CreateResult PowerProvider::createFromSdf(
    const std::string& name,
    const sdf::ElementPtr& sdf,
    rclcpp::Node::SharedPtr node,
    const std::string& vessel_name,
    std::shared_ptr<spdlog::logger> logger)
{
    if (!sdf->HasElement("type")) {
        if (logger)
            logger->error("PowerProvider::createFromSdf: [{}] has <lotusim_power> "
                "but missing required <type> -> skipping", name);
        return {nullptr, ProviderType::Battery};
    }

    const std::string type = sdf->Get<std::string>("type");

    if (type == "simple_battery")
        return {std::make_shared<SimpleBattery>(name, sdf, std::move(node)), ProviderType::Battery};

    if (type == "simple_generator")
        return {std::make_shared<SimpleGenerator>(name, sdf, std::move(node)), ProviderType::Generator};

    if (type == "rpm_generator")
        return {std::make_shared<RpmGenerator>(name, sdf, std::move(node), vessel_name), ProviderType::Generator};

    if (logger)
        logger->error("PowerProvider::createFromSdf: unknown type '{}' for [{}] -> skipping",
            type, name);
    return {nullptr, ProviderType::Battery};
}

} // namespace lotusim::gazebo
