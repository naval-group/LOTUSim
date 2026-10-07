/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#ifndef SECTOR_SCANNER__SECTOR_SCANNER_HPP_
#define SECTOR_SCANNER__SECTOR_SCANNER_HPP_

#include <string>

#include <gz/math/Pose3.hh>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/sector_scan.hpp"

namespace lotusim::sensor {

/**
 * @brief Sector-scanning sensor
 *
 * Sweeps through fixed-width azimuth sectors (default 45 deg), dwelling on each for a configurable time before advancing. 
 * At each tick, reports any detected targets whose bearing falls in the currently active sector and whose range is within max_range.
 *
 * Detection is purely geometric: it iterates every Model entity in the ECM and computes range/bearing directly
 * from world poses.
 *
 * LIMITATIONS (intentional, for now):
 *  - No occlusion: a target directly behind terrain or another hull will still be reported, 
 *    since there's no raycast against the environment.
 *  - No signal strength / target-strength modeling: a contact is either in-sector-and-in-range or it isn't.
 */

class SectorScanner : public CustomSensor {
public:
    SectorScanner(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~SectorScanner() override = default;

    bool CustomSensorLoad(const sdf::Sensor& _sdf) override;

    bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    /// Normalises an angle in degrees to [0, 360)
    static double NormaliseDeg(double deg);

    // ── sector-scan config ────────────────────────────────────────
    double m_sector_width_deg{45.0};
    double m_dwell_time_s{1.0};
    double m_max_range{500.0};
    double m_vertical_beamwidth_deg{30.0};  // full width, centred on the sensor's horizontal plane
    int m_num_sectors{8};

    // ── sector-scan state ─────────────────────────────────────────────────
    int m_current_sector{0};
    double m_sector_start_time_s{-1.0};  // < 0 means "not yet initialised"

    bool m_power_managed{false}; // true if power_manager in the SDF, else active by default

    rclcpp::Publisher<lotusim_sensor_msgs::msg::SectorScan>::SharedPtr m_sector_scan_pub;
};

}  // namespace lotusim::sensor

#endif  // SECTOR_SCANNER__SECTOR_SCANNER_HPP_