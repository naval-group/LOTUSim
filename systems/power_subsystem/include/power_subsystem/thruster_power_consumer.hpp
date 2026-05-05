/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include "power_subsystem/power_consumer.hpp"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>

namespace lotusim::gazebo
{
/**
 * @brief PowerConsumer for thrusters
 *
 * Thrusters drawnCurrent() computes I = (P * rpm_ratio²) / V 
 * using the last voltage received from PowerManager
 */
class ThrusterPowerConsumer final : public PowerConsumer{
public:
    /**
     * @param name      thruster name from SDF name attribute
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param sdf       SDF element for this thruster tag 
     * @param rpm       rpm published from the user example 
     * @param node      node from PowerManager
     * @param ecm       Gazebo EntityComponentManager
     */
    ThrusterPowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        const sdf::ElementPtr& sdf,
        float rpm,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& ecm
    );

    // ----------------------------------------------------------------
    // PowerConsumer interface
    // ----------------------------------------------------------------
    
    /**
     * @brief Returns I = (nominalPowerW() * rpm_ratio²) / m_voltage
     *        Returns 0.0 if inactive or bus voltage is zero
     */
    float drawnCurrent() const override;

    /**
     * @brief override as its a variable duty cycle
     */
    void update() override {}

    /**
     * @brief Deactivates the thruster
     *        calls PowerConsumer::deactivate() to set m_active = false
     */
    void deactivate() override;

};
} //namespace lotusim::gazebo