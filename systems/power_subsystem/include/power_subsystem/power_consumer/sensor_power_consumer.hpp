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

#include <gz/common/Console.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_msgs/srv/activate_sensor.hpp"
#include "power_subsystem/power_consumer/power_consumer.hpp"

namespace lotusim::gazebo {

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
class SensorPowerConsumer final : public PowerConsumer {
public:
    /**
     * @param name      sensor name from SDF
     * @param nominalW  power draw in Watts from SDF nominal_w
     * @param priority  load shedding priority from SDF priority (default 3)
     * @param _sdf      lotus_power SDF element : for user custom params
     * @param node      node from PowerManager
     * @param _ecm      Gazebo ECM
     */
    SensorPowerConsumer(
        std::string name,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<spdlog::logger> logger,
        float nominalW,
        int priority)
        : PowerConsumer(
              std::move(name),
              std::move(node),
              std::move(logger),
              std::move(nominalW),
              std::move(priority))
    {
    }

    void setServiceName(const std::string& vessel_name)
    {
        m_vessel_name = vessel_name;
        const std::string service =
            "/lotusim/" + vessel_name + "/" + name() + "/change_state";
        m_activate_client =
            m_node->create_client<lotusim_sensor_msgs::srv::ActivateSensor>(
                service);
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
    void update() override
    {
        common::PowerStateRegistry::instance().set(
            m_vessel_name + "/" + name(),
            isActive());
    }

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
        m_logger->info("[SensorPowerConsumer] {} deactivated", name());
    }

    /**
     * @brief reactivates this sensor
     *        called by PowerManager when power recovers
     *        TODO: publish /vessel_N/sensor_N = ON so
     *        nodes know this sensor is powered again
     */
    void activate() override
    {
        PowerConsumer::activate();
        callService(true);
        m_logger->info("[SensorPowerConsumer] {} activated", name());
    }

private:
    void callService(bool active)
    {
        if (!m_activate_client || !m_activate_client->service_is_ready()) {
            if (!active)
                m_logger->warn(
                    "[SensorPowerConsumer] {} service not ready, "
                    "registry set only",
                    name());
            return;
        }

        m_logger->info(
            "[SensorPowerConsumer] {} calling change_state active={}",
            name(),
            active);
        auto request = std::make_shared<
            lotusim_sensor_msgs::srv::ActivateSensor::Request>();
        request->activate =
            active;  // confirm field name with ros2 interface show
        m_activate_client->async_send_request(request);
    }

    std::string m_vessel_name;
    rclcpp::Client<lotusim_sensor_msgs::srv::ActivateSensor>::SharedPtr
        m_activate_client;
};
}  // namespace lotusim::gazebo