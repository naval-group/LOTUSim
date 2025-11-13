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
    GetSDFParam<double>(_sdfptr, "m_saturation", m_saturation, 3000);
    GetSDFParam<bool>(_sdfptr, "estimate_depth_on", m_estimate_depth, false);
    GetSDFParam<double>(
        _sdfptr,
        "standard_pressure",
        m_standard_pressure,
        101.325);
    GetSDFParam<double>(_sdfptr, "kPa_per_meter", m_kPa_per_m, 9.80638);
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
    if (!EnableMeasurement(_info.simTime))
        return false;

    double depth = std::abs(m_position.Z());
    double pressure = m_standard_pressure;
    if (depth >= 0) {
        // Convert depth to pressure
        pressure += depth * m_kPa_per_m;
    }

    pressure += GetGaussianNoise(m_noise_amp);
    double inferredDepth = 0.0;
    // Estimate depth, if enabled
    if (m_estimate_depth) {
        inferredDepth = (pressure - m_standard_pressure) / m_kPa_per_m;
    }

    lotusim_sensor_msgs::msg::PressureDepth msg;
    msg.depth = inferredDepth;
    msg.pressure = pressure;
    msg.stddev = m_noise_sigma;

    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor
