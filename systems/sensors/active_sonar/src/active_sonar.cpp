/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "active_sonar/active_sonar.hpp"

#include <cmath>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/common.hpp"

namespace lotusim::sensor {

// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════
ActiveSonar::ActiveSonar(
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

// ═══════════════════════════════════════════════════════════════════════════
// NormaliseDeg - wrap an angle into [0, 360)
// ═══════════════════════════════════════════════════════════════════════════
double ActiveSonar::NormaliseDeg(double deg)
{
    double d = std::fmod(deg, 360.0);
    if (d < 0.0) {
        d += 360.0;
    }
    return d;
}

// ═══════════════════════════════════════════════════════════════════════════
// CustomSensorLoad - read sector/dwell/range config from SDF, create publisher
// ═══════════════════════════════════════════════════════════════════════════
bool ActiveSonar::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr elem = _sdf.Element();

    GetSDFParam<double>(elem, "sector_width_deg", m_sector_width_deg, 45.0);
    GetSDFParam<double>(elem, "dwell_time_s", m_dwell_time_s, 1.0);
    GetSDFParam<double>(elem, "max_range", m_max_range, 500.0);

    if (m_sector_width_deg <= 0.0 || m_sector_width_deg > 360.0) {
        m_logger->warn(
            "ActiveSonar [{}]: invalid sector_width_deg={}, clamping to 45",
            m_sensor_name, m_sector_width_deg);
        m_sector_width_deg = 45.0;
    }
    m_num_sectors = static_cast<int>(std::round(360.0 / m_sector_width_deg));

    m_power_managed = elem->HasElement("lotusim_power");

    m_sonar_pub = m_ros_node->create_publisher<lotusim_sensor_msgs::msg::SonarScan>(
        m_vessel_name + "/" + m_sensor_name + "/scan",
        rclcpp::QoS(10)
    );

    m_logger->info(
        "ActiveSonar [{}]: publisher ready on [{}/{}/scan], "
        "sector_width={}deg ({} sectors), dwell={}s, max_range={}m",
        m_sensor_name,
        m_vessel_name,
        m_sensor_name,
        m_sector_width_deg,
        m_num_sectors,
        m_dwell_time_s,
        m_max_range);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// UpdateSensor - advance sector, scan world entities, publish contacts
// ═══════════════════════════════════════════════════════════════════════════
bool ActiveSonar::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (m_power_managed && !common::PowerStateRegistry::instance().get(
            m_vessel_name + "/" + m_sensor_name)) {
        return false;
    }

    // ── Advance sector based on sim time ──
    const double now_s = std::chrono::duration<double>(_info.simTime).count();
    if (m_sector_start_time_s < 0.0) {
        m_sector_start_time_s = now_s;  // first tick
    } else if (now_s - m_sector_start_time_s >= m_dwell_time_s) {
        m_current_sector = (m_current_sector + 1) % m_num_sectors;
        m_sector_start_time_s = now_s;
    }

    const double sector_lo = m_current_sector * m_sector_width_deg;
    const double sector_hi = sector_lo + m_sector_width_deg;
    const double sector_center = sector_lo + m_sector_width_deg / 2.0;

    auto self_pose = gz::sim::worldPose(m_vessel_entity, _ecm);

    lotusim_sensor_msgs::msg::SonarScan msg;
    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);
    msg.header.frame_id = m_vessel_name + "/" + m_sensor_name;
    msg.current_sector = m_current_sector;
    msg.sector_center_deg = sector_center;
    msg.sector_width_deg = m_sector_width_deg;
    msg.max_range = m_max_range;

    // scans every model entity in the world as a potential target
    auto model_entities = _ecm.EntitiesByComponents(gz::sim::components::Model());
    for (auto&& entity : model_entities) {
        if (entity == m_vessel_entity) {
            continue;  // skip self
        }

        auto name_comp = _ecm.Component<gz::sim::components::Name>(entity);
        const std::string target_name = name_comp ? name_comp->Data() : "unknown";

        auto target_pose = gz::sim::worldPose(entity, _ecm);
        const gz::math::Vector3d rel_world = target_pose.Pos() - self_pose.Pos();
        const gz::math::Vector3d rel_body = self_pose.Rot().RotateVectorReverse(rel_world);

        const double range = rel_body.Length();
        if (range <= 1e-6 || range > m_max_range) {
            continue;
        }

        // Nautical bearing convention: clockwise from bow (0 = ahead)
        // Body frame is front-left-up (FLU), so clockwise-from-bow is the
        // negative of the standard counter-clockwise atan2 angle
        const double bearing_deg = NormaliseDeg(-std::atan2(rel_body.Y(), rel_body.X()) * 180.0 / M_PI);

        const bool in_sector = (sector_lo <= bearing_deg && bearing_deg < sector_hi);
        if (!in_sector) {
            continue;
        }

        lotusim_sensor_msgs::msg::SonarContact contact;
        contact.target_name = target_name;
        contact.range = range;
        contact.bearing_deg = bearing_deg;
        msg.contacts.push_back(contact);
    }

    m_sonar_pub->publish(msg);
    return true;
}

}  // namespace lotusim::sensor