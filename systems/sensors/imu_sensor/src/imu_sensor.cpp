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

    // SDF noise: a small rotation on the orientation (rad), and on each
    // angular velocity (rad/s) and linear acceleration (m/s²) component.
    // The covariances give its variance.
    const gz::math::Quaterniond orientation =
        m_quad * gz::math::Quaterniond(Noise(), Noise(), Noise());
    msg.orientation.x = orientation.X();
    msg.orientation.y = orientation.Y();
    msg.orientation.z = orientation.Z();
    msg.orientation.w = orientation.W();
    const double variance = NoiseStdDev() * NoiseStdDev();
    for (const int i : {0, 4, 8}) {
        msg.orientation_covariance[i] = variance;
        msg.angular_velocity_covariance[i] = variance;
        msg.linear_acceleration_covariance[i] = variance;
    }

    // Vessel velocity is stored on the model entity in the world frame.
    gz::math::Vector3d lin_vel_world;
    gz::math::Vector3d ang_vel_world;
    bool has_lin_vel = false;
    if (auto comp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(
            m_vessel_entity)) {
        lin_vel_world = comp->Data();
        has_lin_vel = true;
    }
    if (auto comp = _ecm.Component<gz::sim::components::WorldAngularVelocity>(
            m_vessel_entity)) {
        ang_vel_world = comp->Data();
    }

    // Angular velocity in the sensor frame.
    const gz::math::Vector3d ang_vel =
        m_quad.RotateVectorReverse(ang_vel_world);
    msg.angular_velocity.x = ang_vel.X() + Noise();
    msg.angular_velocity.y = ang_vel.Y() + Noise();
    msg.angular_velocity.z = ang_vel.Z() + Noise();

    // Linear acceleration from the change in velocity since the last
    // measurement. Zero until two consecutive measurements have a velocity,
    // so the velocity component appearing does not read as a spike.
    const double time = std::chrono::duration<double>(_info.simTime).count();
    gz::math::Vector3d accel_world = gz::math::Vector3d::Zero;
    if (m_has_prev_vel && has_lin_vel) {
        const double dt = time - m_prev_vel_time;
        if (dt > 0.0) {
            accel_world = (lin_vel_world - m_prev_lin_vel) / dt;
        }
    }
    m_prev_lin_vel = lin_vel_world;
    m_prev_vel_time = time;
    m_has_prev_vel = has_lin_vel;

    // Gravity is excluded: the IMU is calibrated on a stationary surface,
    // so a vessel at rest reads zero acceleration.
    const gz::math::Vector3d accel = m_quad.RotateVectorReverse(accel_world);
    msg.linear_acceleration.x = accel.X() + Noise();
    msg.linear_acceleration.y = accel.Y() + Noise();
    msg.linear_acceleration.z = accel.Z() + Noise();

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor