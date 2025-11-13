/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef AIS_SENSOR_HPP
#define AIS_SENSOR_HPP

#include <cmath>
#include <gz/sim/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/ais.hpp"

namespace lotusim::sensor {

class AISSensor : public CustomSensor {
public:
    AISSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~AISSensor();

    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor& _sdf) final;

private:
    // Sensor params
    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    gz::sim::Entity m_base_link;
    rclcpp::Publisher<lotusim_sensor_msgs::msg::AIS>::SharedPtr m_sensor_pub;
};

}  // namespace lotusim::sensor
#endif