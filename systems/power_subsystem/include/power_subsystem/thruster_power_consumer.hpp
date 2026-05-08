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
#include <algorithm>
#include <cmath>
#include <std_msgs/msg/float32.hpp>

namespace lotusim::gazebo
{
/**
 * @brief PowerConsumer for thrusters
 *
 * Thrusters drawnCurrent() computes I = (P * rpm_ratio²) / V 
 * update() reads the current RPM -> received by subscription
 * using the last voltage received from PowerManager
 * Deactivation is purely logical - m_rpmRatio is set to 0.0 
 * so drawnCurrent() returns 0 immediately
 * NOTE: no equivalent of CustomSensor exists for thruster yet
 */
class ThrusterPowerConsumer final : public PowerConsumer{
public:
    /**
     * @param name      thruster name from SDF name attribute
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param _sdf      SDF for max_rpm of the thruster 
     * @param node      node from PowerManager
     * @param _ecm       Gazebo EntityComponentManager
     */
    ThrusterPowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        const sdf::ElementPtr& _sdf,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& /*_ecm*/)
        : PowerConsumer(std::move(name), nominalW, priority, std::move(node))
        , m_maxRpm(_sdf->Get<float>("max_rpm", 1000.0f).first)
    {
        // subscribe to rpm command
        // @TODO: update the topic if needed
        // resolve the Gazebo joint entity for this thruster by name
        const std::string topic = this->name() + "/rpm";
        m_rpmSub = m_node->create_subscription<std_msgs::msg::Float32>(
            topic,
            rclcpp::QoS(10),
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                // ignore RPM commands while deactivated
                if (!isActive()) {
                    return;
                }
                m_rpmRatio = std::clamp(
                    msg->data / m_maxRpm, 0.0f, 1.0f);
            });
        gzmsg << "[ThrusterPowerConsumer] " << this->name()
              << " subscribed to: " << topic << "\n";
 
    }

    // ----------------------------------------------------------------
    // PowerConsumer interface
    // ----------------------------------------------------------------
    
    /**
     * @brief Returns I = (nominalPowerW() * rpm_ratio²) / m_voltage
     *        returns 0.0 if inactive or bus voltage is zero
     */
    float drawnCurrent() const override
    {
        if (!isActive() || m_voltage <= 0.0f) {
            return 0.0f;
        }
        const float actualPowerW = m_nominalPowerW * (m_rpmRatio * m_rpmRatio);
        return actualPowerW / m_voltage;
    }

    /**
     * @brief no op for now
     */
     void update(gz::sim::EntityComponentManager& /*_ecm*/) override {}

    /**
     * @brief Deactivates the thruster
     *        calls PowerConsumer::deactivate() to set m_active = false
     */
    void deactivate() override
    {
        PowerConsumer::deactivate();
        m_rpmRatio = 0.0f;
        gzmsg << "[ThrusterConsumer] " << name()
              << " deactivated (logic only, entity intact, rpm = 0)\n";
    }

    /**
     * @brief reactivates this thruster after deactivation
     *        Sets m_active = true 
     */
    void reactivate() override
    {
        PowerConsumer::reactivate();
        // m_rpmRatio stays at 0.0 until a new rpm message arrives 
        gzmsg << "[ThrusterPowerConsumer] " << name()
              << " reactivated -> awaiting RPM command\n";
    }

    /**
     * @brief Thruster appeared in ECM —> calls reactivate()
     */
    void eachNew() override
    {
        reactivate();
        gzmsg << "[ThrusterConsumer] " << name() << " connected\n";
    }

    /**
     * @brief Thruster removed from ECM —> called by PowerManager via
     *        _ecm.EachRemoved when the thruster joint entity is gone.
     *        Different from deactivate(): entity is physically gone
     */
    void eachDelete() override
    {
        PowerConsumer::deactivate();
        m_rpmRatio = 0.0f;
        gzmsg << "[ThrusterConsumer] " << name()
              << " removed from simulation\n";
    }
private:
    float m_maxRpm{1000.0f};
    float m_rpmRatio{0.0f};
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_rpmSub;
};
} //namespace lotusim::gazebo