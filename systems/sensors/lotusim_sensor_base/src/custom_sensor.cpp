/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_sensor_base/custom_sensor.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim::sensor {
CustomSensor::CustomSensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity& vessel_entity,
    const gz::sim::Entity& sensor_entity,
    const std::string& parent_name,
    const std::string& sensor_name)
    : m_logger(logger)
    , m_vessel_entity(vessel_entity)
    , m_sensor_entity(sensor_entity)
    , m_vessel_name(parent_name)
    , m_sensor_name(sensor_name)
    , m_last_measurement_time(std::chrono::seconds(0))
    , m_is_on(true)
    , m_ros_node(node)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    m_rnd_gen = std::default_random_engine(seed);
}

CustomSensor::~CustomSensor() {}

bool CustomSensor::Load(const sdf::Sensor& _sdf)
{
    gz::sensors::Sensor::Load(_sdf);

    // An optional <seed>, so a noisy run can be repeated. Applied before
    // CustomSensorLoad(), where the noise models draw their bias.
    const sdf::ElementPtr elem = _sdf.Element();
    if (elem && elem->HasElement("seed")) {
        m_rnd_gen.seed(elem->Get<unsigned>("seed"));
    }

    CustomSensorLoad(_sdf);

    m_type = gz::sensors::customType(_sdf);

    sdf::ElementPtr _sdfptr = _sdf.Element();

    bool isSensorOn;
    GetSDFParam<double>(_sdfptr, "update_rate", m_update_rate, 1.0);
    GetSDFParam<bool>(_sdfptr, "is_on", isSensorOn, true);
    m_is_on = isSensorOn;
    if (m_update_rate > 0.0) {
        // Capped at about 30 years, so a tiny rate fits the duration.
        m_update_period = std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::nanoseconds(
            std::llround(std::min(1e9 / m_update_rate, 1e18))));
        m_next_measurement_time = m_update_period;
    }

    m_sensor_activate_service =
        m_ros_node->create_service<lotusim_sensor_msgs::srv::ActivateSensor>(
            m_vessel_name + "/" + m_sensor_name + "/change_state",
            [this, guard = m_cb_guard](
            const std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Request> req,
            std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Response> res) {
                std::lock_guard<std::mutex> lock(guard->mutex);
                if (!guard->alive) {
                    return;
                }
                ChangeSensorState(req, res);
            });
    return true;
}

bool CustomSensor::ChangeSensorState(
    const std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Request>
        _req,
    std::shared_ptr<lotusim_sensor_msgs::srv::ActivateSensor::Response> _res)
{
    m_is_on = _req->activate;
    _res->success = true;
    return true;
}

bool CustomSensor::IsOn()
{
    return m_is_on;
}

bool CustomSensor::Update(const std::chrono::steady_clock::duration&)
{
    return true;
}

void CustomSensor::Position(const gz::math::Vector3d& _pos)
{
    // TODO: to update lat long
    m_position = _pos;
}

void CustomSensor::LatLong(const gz::math::Vector3d& _pos)
{
    // TODO: to update position
    m_lat_long = _pos;
}

void CustomSensor::Orientation(const gz::math::Quaterniond& _quad)
{
    m_quad = _quad;
}

bool CustomSensor::ShouldMeasure(
    const std::chrono::steady_clock::duration& _now)
{
    if (!m_is_on) {
        return false;
    }
    // As in SDF, a rate of 0 or less means: every simulation step.
    if (m_update_rate <= 0.0) {
        return true;
    }
    if (_now < m_next_measurement_time) {
        return false;
    }
    // As Gazebo does: the next one is due a period after this one was, and
    // a sensor that fell behind moves past _now instead of catching up.
    m_next_measurement_time += m_update_period;
    if (m_next_measurement_time <= _now) {
        m_next_measurement_time +=
            ((_now - m_next_measurement_time) / m_update_period + 1) *
            m_update_period;
    }
    return true;
}

}  // namespace lotusim::sensor
