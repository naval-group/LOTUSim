/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "imu_sensor/imu_sensor.hpp"

namespace lotusim::sensor {

IMUSensor::IMUSensor(
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
    , m_update_period(std::chrono::milliseconds(10))
    , m_last_pub(std::chrono::seconds(0))
{
}

IMUSensor::~IMUSensor() {}

bool IMUSensor::CustomSensorLoad(const sdf::Sensor&)
{
    m_sensor_pub = m_ros_node->create_publisher<sensor_msgs::msg::Imu>(
        m_vessel_name + "/" + m_sensor_name + "/" + "IMU",
        rclcpp::QoS(1));
    return true;
}

bool IMUSensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (!EnableMeasurement(_info.simTime))
        return false;

    sensor_msgs::msg::Imu msg;

    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);

    msg.orientation.x = m_quad.X();
    msg.orientation.y = m_quad.Y();
    msg.orientation.z = m_quad.Z();
    msg.orientation.w = m_quad.W();

    auto anglar_vel_opt =
        _ecm.Component<gz::sim::components::AngularVelocity>(m_sensor_entity);
    if (anglar_vel_opt) {
        gz::math::Vector3d ang_vel = anglar_vel_opt->Data();
        msg.angular_velocity.x = ang_vel.X();
        msg.angular_velocity.y = ang_vel.Y();
        msg.angular_velocity.z = ang_vel.Z();
    }

    auto accel_opt = _ecm.Component<gz::sim::components::LinearAcceleration>(
        m_sensor_entity);
    if (accel_opt) {
        gz::math::Vector3d accel = anglar_vel_opt->Data();
        msg.angular_velocity.x = accel.X();
        msg.angular_velocity.y = accel.Y();
        msg.angular_velocity.z = accel.Z();
    }
    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor