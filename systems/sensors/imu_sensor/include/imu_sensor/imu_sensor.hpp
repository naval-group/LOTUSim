/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef IMU_SENSOR_HPP
#define IMU_SENSOR_HPP

#include <array>
#include <memory>

#include "gz/math/Vector3.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_base/noise_model.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace lotusim::sensor {

/**
 * @brief IMU: orientation, angular velocity and linear acceleration of the
 * vessel, in the sensor frame, each with its own noise per axis.
 *
 * Publishes sensor_msgs/msg/Imu on <vessel>/<sensor name>/IMU. The diagonal
 * of each covariance is its axis' noise variance (stddev²); 0 without noise.
 * Gravity is not included in the linear acceleration.
 *
 * SDF, every tag optional unless noted:
 *
 * <sensor name="imu_sensor"          <!-- required; part of the topic -->
 *         type="custom"              <!-- required -->
 *         gz:type="imu">             <!-- required -->
 *
 *   <!-- Read by CustomSensor, as for every custom sensor -->
 *   <update_rate>10</update_rate>    <!-- Hz, default 1; 0 or less: every step -->
 *   <is_on>true</is_on>              <!-- default true; also set at runtime
 *                                         by <vessel>/<sensor>/change_state -->
 *   <seed>42</seed>                  <!-- unsigned; default: from the clock -->
 *
 *   <!-- Noise: Gazebo's IMU tags, read by sdformat's sdf::Imu -->
 *   <imu>
 *     <orientation>                  <!-- LOTUSim only, same form; x y z are
 *                                         roll pitch yaw (rad), applied as a
 *                                         small rotation; precision has no
 *                                         effect here -->
 *       <x>...</x> <y>...</y> <z>...</z>
 *     </orientation>
 *     <angular_velocity>             <!-- rad/s -->
 *       <x>...</x> <y>...</y> <z>...</z>
 *     </angular_velocity>
 *     <linear_acceleration>          <!-- m/s² -->
 *       <x>...</x> <y>...</y> <z>...</z>
 *     </linear_acceleration>
 *   </imu>
 * </sensor>
 *
 * Each <x>, <y>, <z> above holds one <noise>; an axis without it has no noise.
 * All values are in the axis' unit, default 0:
 *
 * <noise type="gaussian">            <!-- none | gaussian | gaussian_quantized -->
 *   <mean>0</mean>                   <!-- added to every measurement -->
 *   <stddev>2e-4</stddev>            <!-- white noise, drawn each measurement -->
 *   <bias_mean>1e-3</bias_mean>      <!-- bias magnitude; sign drawn at load -->
 *   <bias_stddev>1e-4</bias_stddev>  <!-- spread of the bias drawn at load -->
 *   <dynamic_bias_stddev>1e-5</dynamic_bias_stddev>
 *   <dynamic_bias_correlation_time>400</dynamic_bias_correlation_time>
 *                                    <!-- s; with dynamic_bias_stddev, the bias
 *                                         drifts (Gauss-Markov) -->
 *   <precision>1e-4</precision>      <!-- output rounded to a multiple; 0: off -->
 * </noise>
 *
 * See lotusim_sensor_base/noise_model.hpp for the noise model.
 */
class IMUSensor : public CustomSensor {
public:
    IMUSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~IMUSensor();

    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor& _sdf) final;

private:
    // Sensor params
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_sensor_pub;

    /// One noise model per axis: x, y, z (roll, pitch, yaw for orientation);
    /// nullptr for an axis without noise.
    using AxisNoise = std::array<std::unique_ptr<NoiseModel>, 3>;

    /// The value with each axis' noise.
    static gz::math::Vector3d
    Apply(const AxisNoise& noise, const gz::math::Vector3d& value, double dt);

    // From <imu> in the SDF: Gazebo's <angular_velocity> and
    // <linear_acceleration>, and <orientation> (roll, pitch, yaw) in the same
    // form. None by default.
    AxisNoise m_orientation_noise;
    AxisNoise m_angular_velocity_noise;
    AxisNoise m_linear_acceleration_noise;

    // State for differentiating the vessel velocity into acceleration
    gz::math::Vector3d m_prev_lin_vel;
    double m_prev_vel_time{0.0};
    bool m_has_prev_vel{false};
};
}  // namespace lotusim::sensor

#endif
