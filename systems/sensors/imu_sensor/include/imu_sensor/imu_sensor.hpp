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

#include "gz/math/Vector3.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace lotusim::sensor {

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
    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_sensor_pub;

    // State for differentiating the vessel velocity into acceleration
    gz::math::Vector3d m_prev_lin_vel;
    double m_prev_vel_time{0.0};
    bool m_has_prev_vel{false};
};
}  // namespace lotusim::sensor

#endif