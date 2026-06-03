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

#include "power_subsystem/power_consumer.hpp"
#include "lotusim_sensor_msgs/srv/activate_sensor.hpp"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/common/Console.hh>
#include "lotusim_common/common.hpp"

namespace lotusim::gazebo
{

/**
 * @brief PowerConsumer for fixed power draw sensors
 *
 * Sensors draw a stable nominal_w regardless of ops
 * drawnCurrent() simply computes I = P / V using the last voltage
 * received from PowerManager
 *
 * eachNew()    : sensor appeared in ECM -> calls reactivate()
 * eachDelete() : sensor physically removed from simulation -> deactivates
 *
 * SDF:
 *   <sensor name="ais_sensor" type="custom" gz:type="ais">
 *      <lotusim_power>
 *          <power_type>sensor</power_type>
 *          <nominal_w>3.0</nominal_w>
 *          <priority>4</priority>
 *      </lotusim_power>
 *     ...
 *   </sensor>
 */
class SensorPowerConsumer final : public PowerConsumer
{
public:
    /**
     * @param name      sensor name from SDF
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param _sdf      SDF element : for future implementation
     * @param node      node from PowerManager
     * @param _ecm      Gazebo ECM
     */
    SensorPowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        const sdf::ElementPtr& /*_sdf*/,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& /*_ecm*/) 
        : PowerConsumer(std::move(name), nominalW, priority, std::move(node))
    {
        m_logger = logger::createConsoleAndFileLogger(
            "sensor_" + this->name(),
            "sensor_" + this->name() + ".txt");
    }

    void setServiceName(const std::string& vessel_name)
    {
        const std::string service = "/lotusim/" + vessel_name + "/" + name() + "/change_state";
        m_activate_client = m_node->create_client<lotusim_sensor_msgs::srv::ActivateSensor>(service);
    }

    // ----------------------------------------------------------------
    // PowerConsumer interface
    // ----------------------------------------------------------------
    /**
     * @brief Returns I = nominalPowerW() / m_voltage
     *        returns 0.0 if inactive or voltage is zero
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
     *        for sensor active state
     */
    void update(gz::sim::EntityComponentManager& /*_ecm*/) override {}

    /**
     * @brief Deactivates the sensor -> only its power
     *        calls PowerConsumer::deactivate() to set m_active = false
     *        TODO: publish /vessel_N/sensor_N = OFF so
     *        nodes know this sensor is no longer powered
     */
    void deactivate() override
    {
        PowerConsumer::deactivate();
        callService(false);
        m_logger->info( "[SensorPowerConsumer] {} deactivated", name());
    }

    /**
     * @brief reactivates this sensor 
     *        called by PowerManager when power recovers
     *        TODO: publish /vessel_N/sensor_N = ON so
     *        nodes know this sensor is powered again
     */
    void reactivate() override
    {
        PowerConsumer::reactivate();
        callService(true);
        m_logger->info( "[SensorPowerConsumer] {} reactivated", name());
    }

    /**
     * @brief Sensor appeared in ECM —> called by PowerManager via
     *        _ecm.EachNew<gz::sim::components::CustomSensor>()
     *        activate for power draw
     */
    void eachNew() override
    {
        reactivate();
        m_logger->info( "[SensorPowerConsumer] {} connected", name());
    }

    /**
     * @brief Sensor removed from ECM
     *        different from deactivate(): this fires when the sensor entity
     *        is physically gone from the simulation, not just powered off
     */
    void eachDelete() override
    {
        PowerConsumer::deactivate();
        m_logger->info( "[SensorPowerConsumer] {} removed from simulation", name());
    }

private:
    void callService(bool active)
    {
        if (!m_activate_client) {
            m_logger->warn("[SensorPowerConsumer] {} service client not initialised "
                "— call setServiceName() first", name());
            return;
        }
        if (!m_activate_client->service_is_ready()) {
            m_logger->warn("[SensorPowerConsumer] {} service not ready", name());
            return;
        }
        auto request =
            std::make_shared<lotusim_sensor_msgs::srv::ActivateSensor::Request>();
        request->activate = active;  // confirm field name with ros2 interface show
        m_activate_client->async_send_request(request);
    }

    std::shared_ptr<spdlog::logger> m_logger;
    rclcpp::Client<lotusim_sensor_msgs::srv::ActivateSensor>::SharedPtr m_activate_client;
};
} // namespace lotusim::power_subsystem