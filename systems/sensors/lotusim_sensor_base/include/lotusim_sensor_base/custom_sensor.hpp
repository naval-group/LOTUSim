/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_CUSTOM_SENSOR_HPP
#define LOTUSIM_CUSTOM_SENSOR_HPP

#include <atomic>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/System.hh>
#include <mutex>
#include <random>
#include <string>

#include "gz/sensors/Sensor.hh"
#include "lotusim_common/logger.hpp"
#include "lotusim_sensor_base/common.hpp"
#include "lotusim_sensor_msgs/srv/activate_sensor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lotusim::sensor {

class CustomSensor : public gz::sensors::Sensor {
public:
    /// \brief Class constructor
    CustomSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    /// \brief Class destructor
    virtual ~CustomSensor();

    /// \brief Update callback from simulation.
    // virtual bool OnUpdate(const gz::sim::UpdateInfo &info) = 0;

    /// \brief Get status of the sensor
    bool IsOn();

    /// \brief Inherited function. Loading the sensor
    virtual bool Load(const sdf::Sensor& _sdf) override;

    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) = 0;

    bool Update(const std::chrono::steady_clock::duration& _now) override;

    void Position(const gz::math::Vector3d& _pos);

    void LatLong(const gz::math::Vector3d& _pos);

    void Orientation(const gz::math::Quaterniond& _quad);

    // blocks until any ongoing callback finishes, then disables all future ones. Call before destroying the sensor.
    void Shutdown()
    {
        std::lock_guard<std::mutex> lock(m_cb_guard->mutex);
        m_cb_guard->alive = false;
    }

    // getters for power manager to turn sensor off
    const std::string& vesselName() const
    {
        return m_vessel_name;
    }
    const std::string& sensorName() const
    {
        return m_sensor_name;
    }

protected:
    virtual bool CustomSensorLoad(const sdf::Sensor& _sdf) = 0;

    /**
     * @brief Check if it's time to enable measurement or sensor is off
     *
     * @param _now
     * @return true
     * @return false
     */
    bool EnableMeasurement(
        const std::chrono::steady_clock::duration& _now) const;

    // shared with callbacks so they can detect that the sensor was removed
    struct CallbackGuard {
        std::mutex mutex;
        bool alive{true};
    };

private:
    bool ChangeSensorState(
        const std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Request>
            _req,
        std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Response>
            _res);

protected:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    gz::sim::Entity m_vessel_entity;

    gz::sim::Entity m_sensor_entity;

    std::string m_vessel_name;

    std::string m_sensor_name;

    std::shared_ptr<CallbackGuard> m_cb_guard = std::make_shared<CallbackGuard>();

    /// \brief type of sensor loaded
    std::string m_type;

    /// \brief (Simulation) time when the last sensor measurement was
    /// generated.
    std::chrono::steady_clock::duration m_last_measurement_time;

    /// \brief Sensor update rate (Hz); 0 or less: every simulation step
    double m_update_rate;

    /// \brief Pseudo random number generator, shared by the sensor's noise
    /// models; seeded from <seed>, else from the clock
    std::default_random_engine m_rnd_gen;

    /// \brief Flag to control the generation of output messages
    std::atomic<bool> m_is_on;

    rclcpp::Node::SharedPtr m_ros_node;

    rclcpp::Service<lotusim_sensor_msgs::srv::ActivateSensor>::SharedPtr
        m_sensor_activate_service;

    gz::math::Vector3d m_position{std::nan(""), std::nan(""), std::nan("")};

    gz::math::Vector3d m_lat_long{std::nan(""), std::nan(""), std::nan("")};

    gz::math::Quaterniond m_quad{gz::math::Quaterniond::Identity};
};

}  // namespace lotusim::sensor

#endif  // ROS_BASE_PLUGIN_HH
