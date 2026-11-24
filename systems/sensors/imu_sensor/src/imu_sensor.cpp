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

#include <sdf/Imu.hh>
#include <sdf/Noise.hh>

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
{
}

IMUSensor::~IMUSensor() {}

bool IMUSensor::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    // Gazebo's own tags, read by sdformat as for a Gazebo IMU. Without
    // <imu>, or an axis without <noise>, sdf::Noise is of type none.
    sdf::Imu imu;
    std::array<sdf::Noise, 3> orientation_noise;
    const sdf::ElementPtr elem = _sdf.Element();
    if (elem && elem->HasElement("imu")) {
        const sdf::ElementPtr imu_elem = elem->GetElement("imu");
        imu.Load(imu_elem);
        // Gazebo has no orientation noise: <orientation> takes the same form.
        if (imu_elem->HasElement("orientation")) {
            const sdf::ElementPtr orientation =
                imu_elem->GetElement("orientation");
            const char* axes[3] = {"x", "y", "z"};
            for (int i = 0; i < 3; ++i) {
                if (orientation->HasElement(axes[i]) &&
                    orientation->GetElement(axes[i])->HasElement("noise")) {
                    orientation_noise[i].Load(
                        orientation->GetElement(axes[i])->GetElement("noise"));
                }
            }
        }
    }
    auto create = [this](const sdf::Noise& noise) {
        return CreateNoiseModel(noise, m_rnd_gen);
    };
    m_orientation_noise = {
        create(orientation_noise[0]),
        create(orientation_noise[1]),
        create(orientation_noise[2])};
    m_angular_velocity_noise = {
        create(imu.AngularVelocityXNoise()),
        create(imu.AngularVelocityYNoise()),
        create(imu.AngularVelocityZNoise())};
    m_linear_acceleration_noise = {
        create(imu.LinearAccelerationXNoise()),
        create(imu.LinearAccelerationYNoise()),
        create(imu.LinearAccelerationZNoise())};

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

    // Time since the last measurement, for the noise that drifts.
    const double noise_dt =
        std::chrono::duration<double>(_info.simTime - m_last_measurement_time)
            .count();

    // Orientation noise: a small rotation, roll pitch yaw (rad).
    const gz::math::Quaterniond orientation =
        m_quad *
        gz::math::Quaterniond(
            Apply(m_orientation_noise, gz::math::Vector3d::Zero, noise_dt));
    msg.orientation.x = orientation.X();
    msg.orientation.y = orientation.Y();
    msg.orientation.z = orientation.Z();
    msg.orientation.w = orientation.W();
    // The covariances give each axis' variance.
    for (int i = 0; i < 3; ++i) {
        const auto variance = [i](const AxisNoise& noise) {
            return noise[i] ? noise[i]->Variance() : 0.0;
        };
        msg.orientation_covariance[4 * i] = variance(m_orientation_noise);
        msg.angular_velocity_covariance[4 * i] =
            variance(m_angular_velocity_noise);
        msg.linear_acceleration_covariance[4 * i] =
            variance(m_linear_acceleration_noise);
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
    const gz::math::Vector3d ang_vel = Apply(
        m_angular_velocity_noise,
        m_quad.RotateVectorReverse(ang_vel_world),
        noise_dt);
    msg.angular_velocity.x = ang_vel.X();
    msg.angular_velocity.y = ang_vel.Y();
    msg.angular_velocity.z = ang_vel.Z();

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
    const gz::math::Vector3d accel = Apply(
        m_linear_acceleration_noise,
        m_quad.RotateVectorReverse(accel_world),
        noise_dt);
    msg.linear_acceleration.x = accel.X();
    msg.linear_acceleration.y = accel.Y();
    msg.linear_acceleration.z = accel.Z();

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

gz::math::Vector3d IMUSensor::Apply(
    const AxisNoise& noise,
    const gz::math::Vector3d& value,
    double dt)
{
    auto apply = [dt](const std::unique_ptr<NoiseModel>& model, double v) {
        return model ? model->Apply(v, dt) : v;
    };

    return {
        apply(noise[0], value.X()),
        apply(noise[1], value.Y()),
        apply(noise[2], value.Z())};
}

}  // namespace lotusim::sensor
