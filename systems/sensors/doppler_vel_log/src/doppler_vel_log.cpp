/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

 #include "doppler_vel_log/doppler_vel_log.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include "gz/sim/Util.hh"
#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/common.hpp"

namespace lotusim::sensor {
// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════
DopplerVelocityLog::DopplerVelocityLog(
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
// CustomSensorLoad - read beam angle + noise from SDF, create publishers
// ═══════════════════════════════════════════════════════════════════════════
bool DopplerVelocityLog::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr elem = _sdf.Element();

    double beam_angle_deg = 30.0;
    GetSDFParam<double>(elem, "beam_angle_deg", beam_angle_deg, 30);
    m_beam_angle_rad = beam_angle_deg * M_PI / 180.0;
    
    GetSDFParam<double>(elem, "max_altitude", m_max_altitude, 50.0);
    
    // Janus config: 4 beams, 90 deg apart in azimuth
    // m_beam_angle_rad off the vertical (-Z in body frame)
    m_beams.clear();
    for (int i = 0; i < 4; ++i) {
        const double az = i * M_PI / 2.0;
        gz::math::Vector3d dir(
            std::sin(m_beam_angle_rad) * std::cos(az),
            std::sin(m_beam_angle_rad) * std::sin(az),
            -std::cos(m_beam_angle_rad));
        m_beams.push_back({dir});
    }

    m_has_seafloor = LoadSeafloor(elem);
    if(!m_has_seafloor) {
        m_logger->warn("No seafloor configured for DVL sensor '{}'", m_sensor_name);
    }

    m_dvl_pub = m_ros_node->create_publisher<lotusim_sensor_msgs::msg::DVL>(
        m_vessel_name + "/" + m_sensor_name + "/dvl",
        rclcpp::QoS(10)
    );

    if (elem->HasElement("noise")) {
        sdf::Noise noise_sdf;
        noise_sdf.Load(elem->GetElement("noise"));
        m_noise = gz::sensors::NoiseFactory::NewNoiseModel(noise_sdf);
    } else {
        m_logger->warn(
            "DopplerVelocityLog::CustomSensorLoad [{}]: no <noise> configured, "
            "running without noise",
            m_sensor_name);
    }

    m_logger->info(
        "DopplerVelocityLog [{}]: publisher ready on [{}/{}/dvl], "
        "beam_angle={}deg, seafloor={}",
        m_sensor_name,
        m_vessel_name,
        m_sensor_name,
        beam_angle_deg,
        m_has_seafloor ? "loaded" : "none");

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// LoadSeafloor - read seafloor configuration from SDF
// ═══════════════════════════════════════════════════════════════════════════
bool DopplerVelocityLog::LoadSeafloor(sdf::ElementPtr _elem)
{
    if (!_elem->HasElement("lotusim_seafloor")) {
        return false;
    }
    auto seafloor_elem = _elem->GetElement("lotusim_seafloor");
    if(!seafloor_elem->HasElement("png_path")) {
        m_logger->warn("DVL sensor '{}' seafloor config missing <png_path>", m_sensor_name);
        return false;
    }
    std::string png_path = seafloor_elem->Get<std::string>("png_path");
    std::string floor_path = gz::common::findFile(png_path);
    if(floor_path.empty()) {
        m_logger->warn("DVL sensor: seafloor PNG not found: {}", png_path);
        return false;
    }
    if(m_seafloor_image.Load(floor_path) != 0) {
        m_logger->error("DVL sensor: failed to load seafloor PNG: {}", floor_path);
        return false;
    }

    GetSDFParam<double>(seafloor_elem, "size_x", m_size_x, 1000.0);
    GetSDFParam<double>(seafloor_elem, "size_y", m_size_y, 1000.0);
    GetSDFParam<double>(seafloor_elem, "origin_x", m_origin_x, 0.0);
    GetSDFParam<double>(seafloor_elem, "origin_y", m_origin_y, 0.0);
    GetSDFParam<double>(seafloor_elem, "min_depth", m_min_depth, -50.0);
    GetSDFParam<double>(seafloor_elem, "max_depth", m_max_depth, 0.0);

    m_logger->info(
        "DopplerVelocityLog::LoadSeafloor: loaded [{}] ({}x{} px), "
        "world extent {}x{} m, depth range [{}, {}] m.",
        floor_path,
        m_seafloor_image.Width(),
        m_seafloor_image.Height(),
        m_size_x,
        m_size_y,
        m_min_depth,
        m_max_depth);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// SeafloorElevation - get seafloor elevation at a given world position
// ═══════════════════════════════════════════════════════════════════════════
std::optional<double> DopplerVelocityLog::SeafloorElevation(double world_x, double world_y) const
{
    if(!m_has_seafloor || m_size_x <= 0 || m_size_y <= 0) {
        return std::nullopt;
    }

    const double u = (world_x - m_origin_x) / m_size_x;
    const double v = (world_y - m_origin_y) / m_size_y;

    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) {
        return std::nullopt;  // outside the mapped seafloor extent
    }

    const unsigned int width = m_seafloor_image.Width();
    const unsigned int height = m_seafloor_image.Height();
    if (width == 0 || height == 0) {
        return std::nullopt;
    }

    const unsigned int px =
        std::min(width - 1, static_cast<unsigned int>(u * width));
    // Image row 0 = top; flip so v=0 (world_y = origin_y)
    // maps to the image's bottom row, matching top-left origin heightmap
    const unsigned int py =
        std::min(height - 1, static_cast<unsigned int>((1.0 - v) * height));
 
    gz::math::Color pixel = m_seafloor_image.Pixel(px, py);
    const double normalised = pixel.R();  // grayscale
 
    return m_min_depth + normalised * (m_max_depth - m_min_depth);
}


// ═══════════════════════════════════════════════════════════════════════════
// UpdateSensor - get true vel from ECM, project onto beam, add noise, publish
// ═══════════════════════════════════════════════════════════════════════════
bool DopplerVelocityLog::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (!common::PowerStateRegistry::instance().get(
            m_vessel_name + "/" + m_sensor_name)) {
        return false;
    }

    // One-time: find base_link and enable velocity checks on it
    if (m_base_link_entity == gz::sim::kNullEntity) {
        auto children = _ecm.ChildrenByComponents(
            m_vessel_entity,
            gz::sim::components::Link());
        for (auto&& link : children) {
            auto name_opt = _ecm.Component<gz::sim::components::Name>(link);
            if (name_opt && name_opt->Data().find("base_link") != std::string::npos) {
                m_base_link_entity = link;
                break;
            }
        }
        if (m_base_link_entity == gz::sim::kNullEntity) {
            m_logger->warn(
                "DopplerVelocityLog [{}]: base_link not found yet.",
                m_sensor_name);
            return true;
        }
    }

    gz::sim::Link link(m_base_link_entity);
    auto world_pose = gz::sim::worldPose(m_vessel_entity, _ecm);
    auto world_lin_vel = link.WorldLinearVelocity(_ecm);
    if (!world_lin_vel) {
        m_logger->debug(
            "DopplerVelocityLog [{}]: velocity not available this tick.",
            m_sensor_name);
        return true;
    }
 
    // NOTE: no water-current model implemented yet with this, so world-frame velocity is
    // used directly as a stand-in for velocity-relative-to-water
    const gz::math::Vector3d body_vel =
        world_pose.Rot().RotateVectorReverse(*world_lin_vel);
 
    // ── Altitude / bottom lock (if seafloor is configured) ──
    bool bottom_lock = false;
    double altitude = -1.0;
    auto seafloor_z = SeafloorElevation(world_pose.Pos().X(), world_pose.Pos().Y());
    if (seafloor_z) {
        altitude = world_pose.Pos().Z() - *seafloor_z;
        bottom_lock = (altitude > 0.0 && altitude <= m_max_altitude);
    }
 
    lotusim_sensor_msgs::msg::DVL msg;
    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);
    msg.header.frame_id = m_vessel_name + "/" + m_sensor_name;
 
    auto apply_noise = [this](double v) {
        return m_noise ? m_noise->Apply(v) : v;
    };

    msg.velocity.x = apply_noise(body_vel.X());
    msg.velocity.y = apply_noise(body_vel.Y());
    msg.velocity.z = apply_noise(body_vel.Z());
    msg.bottom_lock = bottom_lock;
 
    for (std::size_t i = 0; i < m_beams.size() && i < 4; ++i) {
        const double beam_vel = apply_noise(body_vel.Dot(m_beams[i].direction));
        msg.beam_velocity[i] = beam_vel;
 
        if (bottom_lock) {
            // Locally-flat-seafloor approximation: range along a tilted
            // beam to a flat bottom at the vessel's bottom
            msg.beam_range[i] = altitude / std::cos(m_beam_angle_rad);
        } else {
            msg.beam_range[i] = -1.0;  // no valid range
        }
    }
 
    m_dvl_pub->publish(msg);
    return true;
}



}  // namespace lotusim::sensor