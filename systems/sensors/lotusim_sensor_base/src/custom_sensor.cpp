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

    CustomSensorLoad(_sdf);

    m_type = gz::sensors::customType(_sdf);

    sdf::ElementPtr _sdfptr = _sdf.Element();

    bool isSensorOn;
    GetSDFParam<int>(_sdfptr, "update_rate", m_update_rate, 1);
    GetSDFParam<bool>(_sdfptr, "is_on", isSensorOn, true);
    m_is_on = isSensorOn;

    m_sensor_activate_service =
        m_ros_node->create_service<lotusim_sensor_msgs::srv::ActivateSensor>(
            m_vessel_name + "/" + m_sensor_name + "/change_state",
            std::bind(
                &CustomSensor::ChangeSensorState,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    GetSDFParam<double>(_sdfptr, "noise_sigma", m_noise_sigma, 0.0);
    if (m_noise_sigma >= 0.0) {
        m_logger->warn("Signal noise sigma must be greater or equal to zero");
    }

    GetSDFParam<double>(_sdfptr, "noise_amplitude", m_noise_amp, 0.0);
    if (m_noise_amp >= 0.0) {
        m_logger->warn(
            "Signal noise amplitude must be greater or equal to zero");
    }
    AddNoiseModel("default", m_noise_sigma);
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

double CustomSensor::GetGaussianNoise(std::string _name, double _amp)
{
    if (m_noise_models.count(_name)) {
        m_logger->warn("Gaussian noise model does not exist");
        return 0;
    }
    return _amp * m_noise_models[_name](m_rnd_gen);
}

double CustomSensor::GetGaussianNoise(double _amp)
{
    return _amp * m_noise_models["default"](m_rnd_gen);
}

bool CustomSensor::AddNoiseModel(std::string _name, double _sigma)
{
    if (m_noise_models.count(_name))
        return false;

    m_noise_models[_name] = std::normal_distribution<double>(0.0, _sigma);
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

bool CustomSensor::EnableMeasurement(
    const std::chrono::steady_clock::duration& _now) const
{
    double dt = std::chrono::duration_cast<std::chrono::seconds>(
                    _now - m_last_measurement_time)
                    .count();
    return dt >= 1.0 / m_update_rate && m_is_on;
}

}  // namespace lotusim::sensor
