/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "ais_sensor/ais_sensor.hpp"

namespace lotusim::sensor {

AISSensor::AISSensor(
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
    , m_update_period(std::chrono::seconds(2))
    , m_last_pub(std::chrono::seconds(0))
    , m_base_link(gz::sim::kNullEntity)
{
    m_logger->info(
        "AISSensor::AISSensor: Created for vessel {} sensor {}",
        parent_name,
        sensor_name);
}
AISSensor::~AISSensor() = default;

bool AISSensor::CustomSensorLoad(const sdf::Sensor&)
{
    m_sensor_pub = m_ros_node->create_publisher<lotusim_sensor_msgs::msg::AIS>(
        m_vessel_name + "/" + m_sensor_name + "/" + "ais",
        rclcpp::QoS(1));
    return true;
}

bool AISSensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (m_base_link == gz::sim::kNullEntity) {
        auto child_link = _ecm.ChildrenByComponents(
            m_vessel_entity,
            gz::sim::components::Link());
        for (auto&& link : child_link) {
            auto name_opt = _ecm.Component<gz::sim::components::Name>(link);
            if (name_opt &&
                name_opt->Data().find("base_link") != std::string::npos) {
                m_base_link = link;
                break;
            }
        }
    }

    if (!EnableMeasurement(_info.simTime))
        return false;

    lotusim_sensor_msgs::msg::AIS msg;
    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);

    msg.name = m_vessel_name;
    msg.longitude = m_lat_long.Y();
    msg.latitude = m_lat_long.X();

    auto vel_opt =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(m_base_link);

    if (vel_opt) {
        double vel = std::sqrt(
            std::pow(vel_opt->Data()[0], 2) + std::pow(vel_opt->Data()[1], 2));

        double angleRadians = atan2(
            vel_opt->Data()[0],
            vel_opt->Data()[1]);  // x is East, y is North

        double headingDegrees = angleRadians * 180.0 / M_PI;
        if (headingDegrees < 0) {
            headingDegrees += 360.0;
        }

        msg.sog = vel;
        msg.true_heading = headingDegrees;
    }
    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor
