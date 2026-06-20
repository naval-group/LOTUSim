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
#include "power_subsystem/power_provider/rpm_generator.hpp"
#include "power_subsystem/power_provider/simple_battery.hpp"
#include "power_subsystem/power_provider/simple_generator.hpp"

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
            logger->error(
                "PowerProvider::createFromSdf: [{}] has <lotusim_power> "
                "but missing required <type> -> skipping",
                name);
        return {nullptr, ProviderType::Unknown};
    }

    std::string providerTypeStr = sdf->Get<std::string>("type", "").first;
    const auto typeOpt = providerTypeFromString(providerTypeStr);
    if (!typeOpt) {
        logger->warn(
            "PowerProvider::createFromSdf [{}]: unknown type '{}' on "
            "consumer [{}] -> skipping",
            vessel_name,
            providerTypeStr,
            name);
        return {nullptr, ProviderType::Unknown};
    }

    switch (*typeOpt) {
        case ProviderType::SimpleBattery:
            return {
                std::make_shared<SimpleBattery>(name, sdf, std::move(node)),
                ProviderType::SimpleBattery};

        case ProviderType::SimpleGenerator:
            return {
                std::make_shared<SimpleGenerator>(name, sdf, std::move(node)),
                ProviderType::SimpleGenerator};

        case ProviderType::RPMGenerator:
            return {
                std::make_shared<RpmGenerator>(
                    name,
                    sdf,
                    std::move(node),
                    vessel_name),
                ProviderType::RPMGenerator};
        default:
            logger->error(
                "PowerProvider::createFromSdf: unknown type '{}' for [{}] -> skipping",
                providerTypeStr,
                name);
            return {nullptr, ProviderType::Unknown};
    }
    logger->error(
        "PowerProvider::createFromSdf: unknown type '{}' for [{}] -> skipping",
        providerTypeStr,
        name);
    return {nullptr, ProviderType::Unknown};
}

}  // namespace lotusim::gazebo
