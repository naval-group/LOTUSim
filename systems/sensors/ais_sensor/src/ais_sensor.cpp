/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include <cmath>

#include "ais_sensor/ais_sensor.hpp"

namespace lotusim::sensor {

namespace {

/// \brief Wrap an angle in degrees into [0, 360), the range AIS reports
/// headings and courses in.
double toCompassRange(double _degrees)
{
    double d = std::fmod(_degrees, 360.0);
    return d < 0.0 ? d + 360.0 : d;
}

}  // namespace

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
    if (!EnableMeasurement(_info.simTime))
        return false;

    lotusim_sensor_msgs::msg::AIS msg;
    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);

    msg.name = m_vessel_name;
    msg.longitude = m_lat_long.Y();
    msg.latitude = m_lat_long.X();

    // True heading is where the vessel POINTS, taken from its attitude, and is
    // a different quantity from course over ground below: a vessel holding a
    // track across a current points off that track by the crab angle, and the
    // two readings then differ by exactly that angle. m_quad is the world
    // attitude in ENU, whose yaw runs counter-clockwise from East, so convert
    // it to a compass bearing running clockwise from North.
    msg.true_heading = toCompassRange(90.0 - m_quad.Yaw() * 180.0 / M_PI);

    // The physics plugin keeps the velocity on the model entity.
    auto vel_opt = _ecm.Component<gz::sim::components::WorldLinearVelocity>(
        m_vessel_entity);

    if (vel_opt) {
        // Speed over ground is horizontal by definition: a diving vehicle's
        // vertical rate is not part of it.
        msg.sog = std::sqrt(
            std::pow(vel_opt->Data()[0], 2) + std::pow(vel_opt->Data()[1], 2));

        // Course over ground is where the vessel actually TRAVELS. atan2 of
        // (East, North) is already a compass bearing. It is meaningless at
        // rest, as it is on a real receiver; sog tells a consumer when to
        // disregard it.
        msg.cog = toCompassRange(
            atan2(vel_opt->Data()[0], vel_opt->Data()[1]) * 180.0 / M_PI);
    }
    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}

}  // namespace lotusim::sensor
