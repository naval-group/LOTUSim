/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "radar_sensor/radar_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "lotusim_sensor_base/common.hpp"

namespace lotusim::sensor {

// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════
RadarSensor::RadarSensor(
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
// CustomSensorLoad
// ═══════════════════════════════════════════════════════════════════════════
bool RadarSensor::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr elem = _sdf.Element();

    GetSDFParam<double>(
        elem,
        "psf_ellipse_width",
        m_psf.ellipse_width,
        15000.0);
    GetSDFParam<double>(
        elem,
        "psf_ellipse_height",
        m_psf.ellipse_height,
        5000.0);
    GetSDFParam<double>(elem, "psf_range_factor", m_psf.range_factor, 0.2);
    GetSDFParam<int>(elem, "psf_grid_size", m_psf.grid_size, 900);

    const std::string base = m_vessel_name + "/" + m_sensor_name;

    // Radar topics
    m_radar_img_pub = m_ros_node->create_publisher<sensor_msgs::msg::Image>(
        base + "/radar/image",
        rclcpp::QoS(10));
    m_lidar_img_pub = m_ros_node->create_publisher<sensor_msgs::msg::Image>(
        base + "/radar/lidar_image",
        rclcpp::QoS(10));

    // Structured detections (bonus — not in Python but useful)
    m_radar_scan_pub =
        m_ros_node->create_publisher<lotusim_sensor_msgs::msg::RadarScan>(
            base + "/radar/detections",
            rclcpp::QoS(10));

    m_logger->info(
        "RadarSensor [{}]: publishers ready on [{}]",
        m_sensor_name,
        base);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// OnPointCloud
// ═══════════════════════════════════════════════════════════════════════════
void RadarSensor::OnPointCloud(const gz::msgs::PointCloudPacked& _msg)
{
    std::lock_guard<std::mutex> lock(m_cloud_mutex);
    m_latest_cloud = _msg;
    m_cloud_received = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// UpdateSensor
//  if (first time)  →  subscribe to gz topic lidar_sensor using world name from
//  ECM if (!EnableMeasurement)  →  return false process + publishs
// ═══════════════════════════════════════════════════════════════════════════
bool RadarSensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    // ── One-time setup: subscribe using world name from ECM ───────────────s
    if (!m_subscribed) {
        std::string world_name = lotusim::common::getWorldName(_ecm);
        std::string topic = "world/" + world_name + "/model/" + m_vessel_name +
                            "/link/base_link/sensor/lidar_sensor/scan/points";

        if (!m_gz_node.Subscribe(topic, &RadarSensor::OnPointCloud, this)) {
            m_logger->error(
                "RadarSensor::UpdateSensor: failed to subscribe to [{}]",
                topic);
            return false;
        }
        m_logger->info("RadarSensor::UpdateSensor: subscribed to [{}]", topic);
        m_subscribed = true;
    }
    // m_logger->info(
    //     "RadarSensor::UpdateSensor: tick simTime={} m_is_on={} m_cloud_received={}",
    //     _info.simTime.count(),
    //     m_is_on,
    //     m_cloud_received);

    // ── EnableMeasurement ───────────────────────────────────────────
    if (!EnableMeasurement(_info.simTime))
        return false;

    // ── Grab latest cloud ─────────────────────────────────────────────────
    gz::msgs::PointCloudPacked cloud_copy;
    {
        std::lock_guard<std::mutex> lock(m_cloud_mutex);
        if (!m_cloud_received)
            return true;
        cloud_copy = m_latest_cloud;
        m_cloud_received = false;
    }

    // ── Decode x, y from PointCloudPacked ────────────────────────────────
    int x_offset = -1, y_offset = -1;
    const uint32_t point_step = cloud_copy.point_step();

    for (int i = 0; i < cloud_copy.field_size(); ++i) {
        const auto& f = cloud_copy.field(i);
        if (f.name() == "x")
            x_offset = static_cast<int>(f.offset());
        if (f.name() == "y")
            y_offset = static_cast<int>(f.offset());
    }
    m_logger->info(
        "point_step={} total_bytes={} n_points={}",
        point_step,
        cloud_copy.data().size(),
        cloud_copy.data().size() / point_step);

    if (x_offset < 0 || y_offset < 0) {
        m_logger->warn(
            "RadarSensor [{}]: cloud missing x/y fields",
            m_sensor_name);
        return true;
    }

    const auto& raw = cloud_copy.data();
    const std::size_t n = raw.size() / point_step;

    std::vector<std::array<float, 2>> points;
    points.reserve(n);

    if (n > 0) {
        const auto* ptr = reinterpret_cast<const uint8_t*>(raw.data());
        float x, y;
        std::memcpy(&x, ptr + x_offset, sizeof(float));
        std::memcpy(&y, ptr + y_offset, sizeof(float));
        m_logger->info("  first point raw: x={} y={}", x, y);
    }

    for (std::size_t i = 0; i < n; ++i) {
        const auto* base_ptr =
            reinterpret_cast<const uint8_t*>(raw.data()) + i * point_step;
        float x, y;
        std::memcpy(&x, base_ptr + x_offset, sizeof(float));
        std::memcpy(&y, base_ptr + y_offset, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y))
            continue;
        points.push_back({x, y});
    }

    if (points.empty()) {
        m_logger->warn("RadarSensor [{}]: no valid points", m_sensor_name);
        return true;
    }

    // ── Apply PSF ─────────────────────────────────────────────────────────
    PSFResult result = SimulatePSF(points);

    // ── Build header ──────────────────────────────────────────────────────
    std_msgs::msg::Header header =
        lotusim::common::generateHeaderMessage(_info.simTime);
    header.frame_id = m_vessel_name + "/" + m_sensor_name;

    // ── Publish images  ─────────────────────────────────────────────────────
    PublishImage(
        m_radar_img_pub,
        result.radar_bgr,
        result.width,
        result.height,
        header);
    PublishImage(
        m_lidar_img_pub,
        result.lidar_bgr,
        result.width,
        result.height,
        header);

    // ── Also publish structured RadarScan detections ──────────────────────
    // Derived from points: convert (x,y) → (range, azimuth, intensity=1)
    lotusim_sensor_msgs::msg::RadarScan scan_msg;
    scan_msg.header = header;
    for (auto& p : points) {
        lotusim_sensor_msgs::msg::RadarDetection det;
        det.range = std::sqrt(p[0] * p[0] + p[1] * p[1]);
        det.azimuth = std::atan2(p[1], p[0]);
        det.intensity = 1.0f;
        scan_msg.detections.push_back(det);
    }
    m_radar_scan_pub->publish(scan_msg);

    m_last_measurement_time = _info.simTime;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// SimulatePSF
// ═══════════════════════════════════════════════════════════════════════════
RadarSensor::PSFResult RadarSensor::SimulatePSF(
    const std::vector<std::array<float, 2>>& points) const
{
    const int G = m_psf.grid_size;
    const float Gf = static_cast<float>(G);

    PSFResult result;
    result.width = G;
    result.height = G;

    std::vector<float> radar_grid(G * G, 0.0f);
    std::vector<float> lidar_grid(G * G, 0.0f);

    if (points.empty()) {
        result.radar_bgr.assign(G * G * 3, 0);
        result.lidar_bgr.assign(G * G * 3, 0);
        return result;
    }

    // ── Normalisation  ────────────────────────────────────────────────────
    float min_x = points[0][0], min_y = points[0][1];
    float max_x = min_x, max_y = min_y;
    for (auto& p : points) {
        min_x = std::min(min_x, p[0]);
        min_y = std::min(min_y, p[1]);
        max_x = std::max(max_x, p[0]);
        max_y = std::max(max_y, p[1]);
    }
    const float range_x = std::max(max_x - min_x, 1e-6f);
    const float range_y = std::max(max_y - min_y, 1e-6f);

    const float radar_raw_x = Gf * 0.5f;  // = 450 for G=900
    const float radar_raw_y = Gf * 0.5f;

    struct GridPt {
        float gx, gy;
    };
    std::vector<GridPt> gpts;
    gpts.reserve(points.size());
    for (auto& p : points) {
        float gx = std::round(((p[0] - min_x) / range_x) * (Gf - 1.0f)) + 1.0f;
        float gy = std::round(((p[1] - min_y) / range_y) * (Gf - 1.0f)) + 1.0f;
        gpts.push_back({gx, gy});
    }

    const float ef_w = static_cast<float>(m_psf.ellipse_width);
    const float ef_h = static_cast<float>(m_psf.ellipse_height);
    const float rf = static_cast<float>(m_psf.range_factor);

    for (std::size_t idx = 0; idx < gpts.size(); ++idx) {
        const float gx = gpts[idx].gx;
        const float gy = gpts[idx].gy;

        const float dir_x = gx - radar_raw_x;
        const float dir_y = gy - radar_raw_y;

        const float perp_x = -dir_y;
        const float perp_y = dir_x;

        const float angle_deg =
            std::atan2(perp_y, perp_x) * 180.0f / static_cast<float>(M_PI);
        const float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

        const float distance = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        const float a = std::max(3.0f, std::round(distance * rf * Gf / ef_w));
        const float b = std::max(2.0f, std::round(distance * rf * Gf / ef_h));

        const float cos_a = std::cos(angle_rad);
        const float sin_a = std::sin(angle_rad);

        const int x0 = std::max(0, static_cast<int>(gx - a - b - 1));
        const int x1 = std::min(G - 1, static_cast<int>(gx + a + b + 1));
        const int y0 = std::max(0, static_cast<int>(gy - a - b - 1));
        const int y1 = std::min(G - 1, static_cast<int>(gy + a + b + 1));

        for (int py = y0; py <= y1; ++py)
            for (int px = x0; px <= x1; ++px) {
                const float dx = static_cast<float>(px) - gx;
                const float dy = static_cast<float>(py) - gy;
                const float u = dx * cos_a + dy * sin_a;
                const float v = -dx * sin_a + dy * cos_a;
                if ((u * u) / (b * b) + (v * v) / (a * a) <= 1.0f)
                    radar_grid[py * G + px] = 1.0f;
            }

        const int ix = std::clamp(static_cast<int>(gx), 0, G - 1);
        const int iy = std::clamp(static_cast<int>(gy), 0, G - 1);
        lidar_grid[iy * G + ix] = 1.0f;
    }

    // Green dot at radar origin
    const int dot_x = std::clamp(static_cast<int>(radar_raw_x), 0, G - 1);
    const int dot_y = std::clamp(static_cast<int>(radar_raw_y), 0, G - 1);
    for (int dy = -5; dy <= 5; ++dy)
        for (int dx = -5; dx <= 5; ++dx) {
            if (dx * dx + dy * dy <= 25) {
                int px = std::clamp(dot_x + dx, 0, G - 1);
                int py = std::clamp(dot_y + dy, 0, G - 1);
                radar_grid[py * G + px] = 2.0f;
                lidar_grid[py * G + px] = 2.0f;
            }
        }

    // ── Float grid → BGR uint8 ────────────────────────────────────────────
    result.radar_bgr.resize(G * G * 3);
    result.lidar_bgr.resize(G * G * 3);

    for (int i = 0; i < G * G; ++i) {
        auto write = [](std::vector<uint8_t>& buf, int i, float v) {
            if (v >= 2.0f) {  // green dot
                buf[i * 3 + 0] = 0;
                buf[i * 3 + 1] = 255;
                buf[i * 3 + 2] = 0;
            } else {
                uint8_t c = static_cast<uint8_t>(v * 255.0f);
                buf[i * 3 + 0] = c;  // B
                buf[i * 3 + 1] = c;  // G
                buf[i * 3 + 2] = c;  // R
            }
        };
        write(result.radar_bgr, i, radar_grid[i]);
        write(result.lidar_bgr, i, lidar_grid[i]);
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// PublishImage — pack BGR bytes into sensor_msgs::msg::Image
// ═══════════════════════════════════════════════════════════════════════════
void RadarSensor::PublishImage(
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
    const std::vector<uint8_t>& bgr_data,
    int width,
    int height,
    const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::Image img;
    img.header = header;
    img.height = static_cast<uint32_t>(height);
    img.width = static_cast<uint32_t>(width);
    img.encoding = "bgr8";
    img.step = static_cast<uint32_t>(width * 3);
    img.is_bigendian = false;
    img.data = bgr_data;
    pub->publish(img);
}

}  // namespace lotusim::sensor