/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include "lotusim_common/common.hpp"
#include "power_subsystem/platform_power_manager_base.hpp"

namespace lotusim::gazebo {

class DefaultPlatformPowerManager : public PlatformPowerManagerBase {
public:
    DefaultPlatformPowerManager(
        const gz::sim::Entity& vessel_entity,
        const std::string& vessel_name,
        rclcpp::Node::SharedPtr node,
        sdf::ElementPtr sdfptr);

private:
    std::shared_ptr<Generator> firstActiveGenerator();

    void handlePowerUpdate(float dt) final;

    void distributeLoad(float dt, float total_current_a, float bus_voltage);

    bool handleDepleted(float dt, float& bus_voltage);

    void shedLoads(PowerLevel level);

    float computeChargeCurrentA(
        std::shared_ptr<Generator> gen,
        std::shared_ptr<Battery> bat,
        float safe_voltage) const;

    void reactivateIfPossible(float available_w, float bus_voltage);
};

}  // namespace lotusim::gazebo
