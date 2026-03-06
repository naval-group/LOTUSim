/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <gz/msgs/pointcloud_packed.pb.h>

#include <array>
#include <gz/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/radar_detection.hpp"
#include "lotusim_sensor_msgs/msg/radar_scan.hpp"

namespace lotusim::sensor {

/**
 * @brief PSF tuning parameters, loaded from SDF at sensor creation.
 *
 */
struct RadarPSFParams {
    double ellipse_width{
        15000.0};  ///< Along-range ellipse divisor — higher = smaller smear
    double ellipse_height{
        5000.0};  ///< Cross-range ellipse divisor — higher = smaller smear
    double range_factor{0.2};  ///< Linear range/size dependency: farther
                               ///< targets → bigger ellipse
    int grid_size{
        900};  ///< Image resolution in pixels (NxN). Python default: 900
};

/**
 * @brief Radar sensor plugin for LOTUSim.
 *
 * Converts the raw Gazebo GPU-LiDAR PointCloudPacked into two radar-like
 * images by applying a Point-Spread-Function (PSF) smearing model.
 *
 * Data flow
 * ─────────
 *  GPU-LiDAR (Gazebo)
 *      │  gz::msgs::PointCloudPacked  (scan/points topic)
 *      ▼
 *  OnPointCloud()  ──cached under mutex──►  m_latest_cloud
 *      │
 *  UpdateSensor() [called every PostUpdate tick at update_rate Hz]
 *      │
 *  SimulatePSF()  ──►  radar PSF image  +  raw lidar hit image
 *      │
 *  PublishImage() ──►  ROS 2 sensor_msgs/Image  ×2
 *                 ──►  ROS 2 lotusim_sensor_msgs/RadarScan (range/azimuth)
 *
 * Integration
 * ───────────
 *  LotusimSensorPlugin recognises gz:type="radar" and calls
 *  CreateSensor<RadarSensor>(...). No extra parameters are needed — the gz
 *  topic is constructed automatically from the world name (read from ECM on
 *  the first UpdateSensor tick) and the vessel name.
 *
 * Published ROS 2 topics (prefix: <vessel_name>/<sensor_name>)
 * ─────────────────────────────────────────────────────────────
 *  /radar/image        sensor_msgs/Image  — PSF-smeared radar display (bgr8)
 *  /radar/lidar_image  sensor_msgs/Image  — raw lidar hit map (bgr8)
 *  /radar/detections   lotusim_sensor_msgs/RadarScan — (range, azimuth,
 * intensity)
 */

class RadarSensor : public CustomSensor {
public:
    RadarSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~RadarSensor() override = default;

    /**
     * @brief Load PSF parameters from SDF and create ROS 2 publishers.
     */
    bool CustomSensorLoad(const sdf::Sensor& _sdf) override;

    /**
     * @brief Called every PostUpdate tick by LotusimSensorPlugin.
     *
     * On the very first call, subscribes to the GPU-LiDAR gz topic.
     * Subsequent calls process the slatest cached PointCloudPacked and publish
     * radar images.
     */
    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief Gazebo transport callback — caches the latest PointCloudPacked.
     *
     * Called from a gz transport thread. Access to m_latest_cloud is
     * protected by m_cloud_mutex.
     */
    void OnPointCloud(const gz::msgs::PointCloudPacked& _msg);

    /**
     * @brief PSFResult struct to hold the results of PSF simulation.
     *
     * For each (x,y) point:
     *   1. Normalise world coordinates onto a grid_size × grid_size grid.
     *   2. Compute direction from radar origin (grid centre) to the point.
     *   3. Draw a filled rotated ellipse whose semi-axes grow with distance
     *      (range_factor). This replaces cv2.ellipse() from the Python code.
     *   4. Mark a single pixel on the lidar image (replaces cv2.circle r=1).
     * Finally, draw a green dot at the radar origin on both images.
     *
     * @param points  Nx2 array of (x, y) in sensor frame (metres)
     * @return radar BGR image and lidar BGR image (grid_size x grid_size x 3)
     */
    struct PSFResult {
        std::vector<uint8_t> radar_bgr;
        std::vector<uint8_t> lidar_bgr;
        int width{0};
        int height{0};
    };
    PSFResult SimulatePSF(
        const std::vector<std::array<float, 2>>& points) const;

    /**
     * @brief Pack a BGR byte array into a sensor_msgs::msg::Image and publish.
     */
    void PublishImage(
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
        const std::vector<uint8_t>& bgr_data,
        int width,
        int height,
        const std_msgs::msg::Header& header);

    // ── gz transport ─────────────────────────────────────────────────────
    gz::transport::Node m_gz_node;

    /**
     * @brief Guards the one-time gz topic subscription.
     */
    bool m_subscribed{false};

    // ── Latest cloud (protected by mutex) ──────────────────────────────────
    std::mutex m_cloud_mutex;
    gz::msgs::PointCloudPacked m_latest_cloud;
    bool m_cloud_received{false};

    // ── PSF params ────────────────────────────────────────────────────────
    RadarPSFParams m_psf;

    // ── ROS 2 publishers ─────────────────────────────────────────────────

    /// PSF-smeared radar display image → <vessel>/<sensor>/radar/image
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_radar_img_pub;

    /// Raw lidar hit image → <vessel>/<sensor>/radar/lidar_image
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_lidar_img_pub;

    /// Structured polar detections → <vessel>/<sensor>/radar/detections
    rclcpp::Publisher<lotusim_sensor_msgs::msg::RadarScan>::SharedPtr
        m_radar_scan_pub;
};

}  // namespace lotusim::sensor