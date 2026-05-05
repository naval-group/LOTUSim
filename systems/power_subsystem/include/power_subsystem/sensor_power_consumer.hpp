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
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/common/Console.hh>

namespace lotusim::gazebo
{

/**
 * @brief PowerConsumer for sensors -> fixed power draw
 *
 * Sensors draw a stable nominal_w regardless of what it is doing
 * drawnCurrent() simply computes I = P / V using the last voltage
 * received from PowerManager
 *
 * SDF:
 *   <sensor name="ais_sensor" type="custom" gz:type="ais"
 *           power_type="sensor" nominal_w="5.0" priority="3">
 *     ...
 *   </sensor>
 */
class SensorPowerConsumer final : public PowerConsumer
{
public:
    /**
     * @param name      sensor name from SDF name attribute
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param sdf       SDF element for this sensor tag 
     * @param node      node from PowerManager
     * @param ecm       Gazebo EntityComponentManager
     */
    SensorPowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& ecm) 
        : PowerConsumer(std::move(name), nominalW, priority, std::move(node)){}

    // ----------------------------------------------------------------
    // PowerConsumer interface
    // ----------------------------------------------------------------
    /**
     * @brief Returns I = nominalPowerW() / m_voltage
     *        Returns 0.0 if inactive or bus voltage is zero
     */
    float drawnCurrent() const override
    {
        if (!isActive() || m_voltage <= 0.0f) {
            return 0.0f;
        }
        return m_nominalPowerW / m_voltage;
    }

    /**
     * @brief No-op for a fixed-draw sensor
     *        override in future if variable duty cycle is needed
     */
    void update(gz::sim::EntityComponentManager& ecm) override {}

    /**
     * @brief Deactivates the sensor
     *        calls PowerConsumer::deactivate() to set m_active = false
     */
    void deactivate() override
    {
        PowerConsumer::deactivate();
        gzmsg << "[SensorConsumer] " << name()
              << " deactivated (logical only — entity intact)\n";
    }

    /**
     * @brief Sensor appeared in ECM —> called by PowerManager via
     *        _ecm.EachNew<gz::sim::components::CustomSensor>()
     *        Re-activates the consumer if it was previously deactivated,
     *        for future reactivation logic
     */
    void eachNew() override
    {
        m_active = true;
        gzmsg << "[SensorConsumer] " << name() << " connected\n";
    }

    /**
     * @brief Sensor removed from ECM —> called by PowerManager via
     *        _ecm.EachRemoved<gz::sim::components::CustomSensor>()
     *        Different from deactivate(): this fires when the sensor entity
     *        is physically gone from the simulation, not just powered off
     */
    void eachDelete() override
    {
        PowerConsumer::deactivate();
        gzmsg << "[SensorConsumer] " << name()
              << " removed from simulation\n";
    }

};

} // namespace lotusim::power_subsystem