/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "subsea_pressure_sensor/subsea_pressure_sensor.hh"

#include <algorithm>
#include <cmath>
#include <sdf/Noise.hh>

namespace lotusim::sensor {

SubseaPressureSensor::SubseaPressureSensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity& vessel_entity,
    const gz::sim::Entity& sensor_entity,
    const std::string& parent_name,
    const std::string& sensor_name)
    : CustomSensor(
          logger,
          node,
          vessel_entity,
          sensor_entity,
          parent_name,
          sensor_name)
{
}

SubseaPressureSensor::~SubseaPressureSensor() {}

bool SubseaPressureSensor::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr _sdfptr = _sdf.Element();
    // <saturation>, or <m_saturation> as SDFs named it before.
    if (!GetSDFParam<double>(_sdfptr, "saturation", m_saturation, 3000.0)) {
        GetSDFParam<double>(_sdfptr, "m_saturation", m_saturation, 3000.0);
    }
    GetSDFParam<bool>(_sdfptr, "estimate_depth_on", m_estimate_depth, false);
    GetSDFParam<double>(
        _sdfptr,
        "standard_pressure",
        m_standard_pressure,
        101.325);
    GetSDFParam<double>(_sdfptr, "kPa_per_meter", m_kPa_per_m, 9.80638);
    // In the form of Gazebo's air pressure sensor: <pressure><noise>.
    sdf::Noise pressure_noise;
    if (_sdfptr && _sdfptr->HasElement("pressure") &&
        _sdfptr->GetElement("pressure")->HasElement("noise")) {
        pressure_noise.Load(
            _sdfptr->GetElement("pressure")->GetElement("noise"));
    }
    m_pressure_noise = CreateNoiseModel(pressure_noise, m_rnd_gen);
    m_sensor_pub =
        m_ros_node->create_publisher<lotusim_sensor_msgs::msg::PressureDepth>(
            m_vessel_name + "/" + m_sensor_name + "/pressure_sensor",
            rclcpp::QoS(1));
    return true;
}

bool SubseaPressureSensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager&)
{
    // Need to rewrite
    if (!ShouldMeasure(_info.simTime))
        return false;

    double depth = std::abs(m_position.Z());
    double pressure = m_standard_pressure;
    if (depth >= 0) {
        // Convert depth to pressure
        pressure += depth * m_kPa_per_m;
    }

    if (m_pressure_noise) {
        const double dt = std::chrono::duration<double>(
                              _info.simTime - m_last_measurement_time)
                              .count();
        pressure = m_pressure_noise->Apply(pressure, dt);
    }
    // The most the sensor reads, noise included.
    pressure = std::min(pressure, m_saturation);
    double inferredDepth = 0.0;
    // Estimate depth, if enabled
    if (m_estimate_depth) {
        inferredDepth = (pressure - m_standard_pressure) / m_kPa_per_m;
    }

    lotusim_sensor_msgs::msg::PressureDepth msg;
    msg.depth = inferredDepth;
    msg.pressure = pressure;
    msg.stddev =
        m_pressure_noise ? std::sqrt(m_pressure_noise->Variance()) : 0.0;

    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor
