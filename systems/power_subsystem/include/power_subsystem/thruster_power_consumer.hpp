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

namespace lotusim::gazebo
{
/**
 * @brief PowerConsumer for thrusters
 *
 * Thrusters drawnCurrent() computes I = (P * rpm_ratio²) / V 
 * update() reads the current RPM
 * using the last voltage received from PowerManager
 * Deactivation is purely logical - m_rpmRatio is set to 0.0 
 * so drawnCurrent() returns 0 immediately
 */
class ThrusterPowerConsumer final : public PowerConsumer{
public:
    /**
     * @param name      thruster name from SDF name attribute
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param sdf       SDF element for this thruster tag 
     * @param node      node from PowerManager
     * @param ecm       Gazebo EntityComponentManager
     */
    ThrusterPowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& ecm)
        : PowerConsumer(std::move(name), nominalW, priority, std::move(node))
        , m_maxRpm(_sdf->Get<float>("max_rpm", 1000.0f))
    {
        // resolve the Gazebo joint entity for this thruster by name
        _ecm.Each<gz::sim::components::Name,
                  gz::sim::components::JointVelocity>(
            [&](const gz::sim::Entity& _entity,
                const gz::sim::components::Name* _nameComp,
                const gz::sim::components::JointVelocity*) -> bool
            {
                if (_nameComp->Data() == this->name()) {
                    m_thrusterEntity = _entity;
                    return false; // stop iteration — > found it
                }
                return true;
            });
 
        if (m_thrusterEntity == gz::sim::kNullEntity) {
            gzwarn << "[ThrusterConsumer] could not find joint entity for: "
                   << this->name()
                   << " — RPM will read as 0 until entity is found\n";
        }
    }

    // ----------------------------------------------------------------
    // PowerConsumer interface
    // ----------------------------------------------------------------
    
    /**
     * @brief Returns I = (nominalPowerW() * rpm_ratio²) / m_voltage
     *        Returns 0.0 if inactive or bus voltage is zero
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
     * @brief Reads current RPM from Gazebo ECM and updates m_rpmRatio
     *        if the thruster was deactivated, m_rpmRatio stays at 0
     */
    void update(gz::sim::EntityComponentManager& _ecm) override
    {
        // if deactivated, hold rpm at zero regardless of ECM state
        if (!isActive()) {
            m_rpmRatio = 0.0f;
            return;
        }
 
        if (m_thrusterEntity == gz::sim::kNullEntity || m_maxRpm <= 0.0f) {
            return;
        }
 
        const auto* velComp =
            _ecm.Component<gz::sim::components::JointVelocity>(
                m_thrusterEntity);
 
        if (velComp && !velComp->Data().empty()) {
            // JointVelocity data is in RPM directly ?????
            const float currentRpm =
                static_cast<float>(velComp->Data()[0]);
            m_rpmRatio = std::clamp(currentRpm / m_maxRpm, 0.0f, 1.0f);
        }
    }

    /**
     * @brief Deactivates the thruster
     *        calls PowerConsumer::deactivate() to set m_active = false
     */
    void deactivate() override
    {
        PowerConsumer::deactivate();
        m_rpmRatio = 0.0f;
        gzmsg << "[ThrusterConsumer] " << name()
              << " deactivated (logical only, entity intact, rpm = 0)\n";
    }

    /**
     * @brief Thruster appeared in ECM —> called by PowerManager via
     *        _ecm.EachNew when a thruster joint component is detected
     *        Re-activates the consumer and allows RPM polling to resume
     */
    void eachNew() override
    {
        m_active = true;
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
        m_thrusterEntity = gz::sim::kNullEntity;
        gzmsg << "[ThrusterConsumer] " << name()
              << " removed from simulation\n";
    }
private:
    float m_maxRpm{1000.0f};
    float m_rpmRatio{0.0f};
    gz::sim::Entity m_thrusterEntity{gz::sim::kNullEntity};
};
} //namespace lotusim::gazebo