/**
 * @file seakeeping_model.hpp
 * @brief Ship seakeeping and motion prediction model
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#ifndef LOTUSIM_ENVIRONMENT_VEHICLE_SEAKEEPING_MODEL_HPP_
#define LOTUSIM_ENVIRONMENT_VEHICLE_SEAKEEPING_MODEL_HPP_

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace lotusim {
namespace environment {

/**
 * @brief Ship motion types (6 degrees of freedom)
 */
enum class MotionType
{
    SURGE = 0,  // Longitudinal (bow-stern)
    SWAY = 1,   // Lateral (port-starboard)
    HEAVE = 2,  // Vertical (up-down)
    ROLL = 3,   // Rotation about longitudinal axis
    PITCH = 4,  // Rotation about lateral axis
    YAW = 5     // Rotation about vertical axis
};

/**
 * @brief Response Amplitude Operator (RAO) data
 *
 * RAO represents the ship's motion response per unit wave amplitude
 * as a function of wave frequency and heading
 */
struct RAOData {
    std::vector<double> frequencies_rad_s;  // Wave frequencies (rad/s)
    std::vector<double> headings_deg;       // Wave headings (deg, 0=head seas)
    std::map<MotionType, std::vector<std::vector<double>>>
        amplitudes;  // Response amplitudes [freq][heading]
    std::map<MotionType, std::vector<std::vector<double>>>
        phases;  // Response phases [freq][heading]
};

/**
 * @brief Ship motion state (6-DOF)
 */
struct ShipMotions {
    // Linear motions (m)
    double surge{0.0};
    double sway{0.0};
    double heave{0.0};

    // Angular motions (rad)
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};

    // Linear velocities (m/s)
    double surge_velocity{0.0};
    double sway_velocity{0.0};
    double heave_velocity{0.0};

    // Angular velocities (rad/s)
    double roll_velocity{0.0};
    double pitch_velocity{0.0};
    double yaw_velocity{0.0};

    // Linear accelerations (m/s²)
    double surge_acceleration{0.0};
    double sway_acceleration{0.0};
    double heave_acceleration{0.0};

    // Angular accelerations (rad/s²)
    double roll_acceleration{0.0};
    double pitch_acceleration{0.0};
    double yaw_acceleration{0.0};
};

/**
 * @brief Acceleration at a specific location on the ship
 */
struct LocationAcceleration {
    Eigen::Vector3d linear_accel;   // Linear acceleration (m/s²)
    Eigen::Vector3d angular_accel;  // Angular acceleration (rad/s²)
    double total_magnitude;         // Total acceleration magnitude (m/s²)
};

/**
 * @brief Operability criteria
 */
struct OperabilityCriteria {
    // Motion limits
    double max_roll_deg{15.0};         // Maximum roll angle
    double max_pitch_deg{10.0};        // Maximum pitch angle
    double max_heave_m{3.0};           // Maximum heave amplitude
    double max_roll_rate_deg_s{5.0};   // Maximum roll rate
    double max_pitch_rate_deg_s{5.0};  // Maximum pitch rate

    // Acceleration limits
    double max_vertical_accel_g{0.3};      // Max vertical acceleration
    double max_lateral_accel_g{0.2};       // Max lateral acceleration
    double max_longitudinal_accel_g{0.2};  // Max longitudinal acceleration

    // Motion comfort
    double max_msi_percent{20.0};  // Max Motion Sickness Incidence

    // Environmental limits
    double max_sea_state{6};  // Maximum operable sea state
    double max_wind_speed_m_s{25.0};
};

/**
 * @brief Operability assessment result
 */
struct OperabilityStatus {
    bool is_operable{true};
    std::vector<std::string> limiting_factors;  // Why not operable
    double overall_score{1.0};                  // 0-1, 1=fully operable
    double msi_percent{0.0};  // Motion Sickness Incidence percentage
};

/**
 * @brief Seakeeping model for ship motion prediction
 */
class SeakeepingModel {
public:
    SeakeepingModel();

    /**
     * @brief Set the RAO data for a vessel type
     */
    void setRAO(const std::string& vessel_type, const RAOData& rao);

    /**
     * @brief Get RAO for a specific vessel type
     */
    const RAOData& getRAO(const std::string& vessel_type) const;

    /**
     * @brief Compute ship motions from sea state
     *
     * @param vessel_type Type of vessel
     * @param significant_wave_height Significant wave height (m)
     * @param peak_period Wave peak period (s)
     * @param wave_heading Wave heading relative to ship (deg, 0=head seas)
     * @param ship_speed Ship speed (m/s)
     * @return Ship motions (6-DOF)
     */
    ShipMotions computeMotions(
        const std::string& vessel_type,
        double significant_wave_height,
        double peak_period,
        double wave_heading,
        double ship_speed) const;

    /**
     * @brief Compute acceleration at a specific location on the ship
     *
     * @param motions Current ship motions
     * @param position Position relative to ship center (m)
     * @return Acceleration at that location
     */
    LocationAcceleration computeLocationAcceleration(
        const ShipMotions& motions,
        const Eigen::Vector3d& position) const;

    /**
     * @brief Compute Motion Sickness Incidence (MSI)
     *
     * @param motions Current ship motions
     * @param exposure_time_hours Exposure time (hours)
     * @return MSI percentage (0-100%)
     */
    double computeMSI(const ShipMotions& motions, double exposure_time_hours)
        const;

    /**
     * @brief Assess operability based on criteria
     *
     * @param motions Current ship motions
     * @param criteria Operability criteria
     * @param sea_state Current sea state
     * @param wind_speed Current wind speed (m/s)
     * @return Operability status
     */
    OperabilityStatus assessOperability(
        const ShipMotions& motions,
        const OperabilityCriteria& criteria,
        int sea_state,
        double wind_speed) const;

    /**
     * @brief Create standard RAO for a vessel class
     */
    static RAOData createStandardRAO(const std::string& vessel_class);

    /**
     * @brief Get standard operability criteria for a vessel type
     */
    static OperabilityCriteria getStandardCriteria(
        const std::string& vessel_type);

private:
    std::map<std::string, RAOData> rao_database_;

    /**
     * @brief Compute single motion component from RAO
     */
    double computeSingleMotion(
        const RAOData& rao,
        MotionType motion_type,
        double wave_amplitude,
        double wave_frequency,
        double wave_heading,
        double time,
        double ship_speed) const;

    /**
     * @brief Interpolate RAO value
     */
    double interpolateRAO(
        const std::vector<double>& frequencies,
        const std::vector<double>& headings,
        const std::vector<std::vector<double>>& values,
        double freq,
        double heading) const;

    /**
     * @brief Compute encounter frequency
     */
    double computeEncounterFrequency(
        double wave_frequency,
        double wave_heading,
        double ship_speed) const;
};

}  // namespace environment
}  // namespace lotusim

#endif  // LOTUSIM_ENVIRONMENT_VEHICLE_SEAKEEPING_MODEL_HPP_
