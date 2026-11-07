/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <array>
#include <string>
#include <vector>
 
#include <gz/common/Image.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/dvl.hpp"

namespace lotusim::sensor {

/**
 * @brief Doppler Velocity Log sensor
 *
 * Reports vessel velocity relative to the world frame, projected onto 4 beams. 
 * Can also samples a grayscale heightmap ("lotusim_seafloor") to estimate altitude 
 * and per-beam range for bottom-lock state. Currently, example is default to max 350m elevation from seafloor.
 *
 * LIMITATIONS (intentional, for now):
 *  - No water current simulation: "velocity relative to world" is used as a
 *    stand-in for "velocity relative to water".
 *    Later: implement a water current field and subtract it from the world-relative velocity
 *    before projecting onto beams.
 *  - Bottom range is computed by sampling the heightmap directly below the
 *    vessel and assuming a locally flat seafloor under each beam.
 */

class DopplerVelocityLog : public CustomSensor {
public:
    DopplerVelocityLog(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~DopplerVelocityLog() override = default;

    bool CustomSensorLoad(const sdf::Sensor& _sdf) override;

    bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
    * @brief Load the grayscale seafloor from the <lotusim_seafloor>
    * SDF if present
    *
    * @param _elem
    * @return true if a valid seafloor was configured
    */
    bool LoadSeafloor(sdf::ElementPtr _elem);

    /**
     * @brief Sample seafloor elevation using nearest-pixel lookup
     * on the heightmap
     *
     * @return seafloor z in world frame, or nullopt if (x, y) no
     * seafloor is configured
     */
    std::optional<double> SeafloorElevation(double world_x, double world_y) const;

    // ── Beam configuration (4 beams) ────────────────────────────────
    struct Beam {
        gz::math::Vector3d direction;
    };
    std::vector<Beam> m_beams;
    double m_beam_angle_rad{0.0};
    double m_max_altitude{350.0};
    gz::sensors::NoisePtr m_noise;
 
    // ── Seafloor heightmap ──────────────────────────────────────────────────
    bool m_has_seafloor{false};
    gz::common::Image m_seafloor_image;
    double m_size_x{0.0};
    double m_size_y{0.0};
    double m_origin_x{0.0};
    double m_origin_y{0.0};
    double m_min_depth{0.0};
    double m_max_depth{0.0};

    bool m_seafloor_origin_explicit{false};
    bool m_seafloor_centered{false};

    gz::sim::Entity m_base_link_entity{gz::sim::kNullEntity};

    rclcpp::Publisher<lotusim_sensor_msgs::msg::DVL>::SharedPtr m_dvl_pub;
};

} // namespace lotusim::sensor