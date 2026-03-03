/**
 * @file seakeeping_model.cpp
 * @brief Ship seakeeping implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/vehicle/seakeeping_model.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim {
namespace environment {

// ============================================================================
// SeakeepingModel Implementation
// ============================================================================

SeakeepingModel::SeakeepingModel()
{
    // Initialize with standard RAOs for common vessel types
    rao_database_["frigate"] = createStandardRAO("frigate");
    rao_database_["corvette"] = createStandardRAO("corvette");
    rao_database_["patrol_boat"] = createStandardRAO("patrol_boat");
}

void SeakeepingModel::setRAO(const std::string& vessel_type, const RAOData& rao)
{
    rao_database_[vessel_type] = rao;
}

const RAOData& SeakeepingModel::getRAO(const std::string& vessel_type) const
{
    auto it = rao_database_.find(vessel_type);
    if (it != rao_database_.end()) {
        return it->second;
    }

    // Default to frigate
    return rao_database_.at("frigate");
}

ShipMotions SeakeepingModel::computeMotions(
    const std::string& vessel_type,
    double significant_wave_height,
    double peak_period,
    double wave_heading,
    double ship_speed) const
{
    ShipMotions motions;

    // Get RAO for vessel
    const RAOData& rao = getRAO(vessel_type);

    // Wave parameters
    double wave_frequency = 2.0 * M_PI / peak_period;  // Peak frequency (rad/s)
    double wave_amplitude = significant_wave_height / 2.0;  // Simplified

    // Encounter frequency (accounting for ship speed)
    double encounter_freq =
        computeEncounterFrequency(wave_frequency, wave_heading, ship_speed);

    // Current time (for phase)
    double time = 0.0;  // Simplified, should be actual simulation time

    // Compute each motion component using RAO
    motions.surge = computeSingleMotion(
        rao,
        MotionType::SURGE,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);
    motions.sway = computeSingleMotion(
        rao,
        MotionType::SWAY,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);
    motions.heave = computeSingleMotion(
        rao,
        MotionType::HEAVE,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);
    motions.roll = computeSingleMotion(
        rao,
        MotionType::ROLL,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);
    motions.pitch = computeSingleMotion(
        rao,
        MotionType::PITCH,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);
    motions.yaw = computeSingleMotion(
        rao,
        MotionType::YAW,
        wave_amplitude,
        wave_frequency,
        wave_heading,
        time,
        ship_speed);

    // Compute velocities (simplified as harmonic derivatives)
    motions.heave_velocity = -wave_amplitude * encounter_freq *
                             std::cos(encounter_freq * time) *
                             interpolateRAO(
                                 rao.frequencies_rad_s,
                                 rao.headings_deg,
                                 rao.amplitudes.at(MotionType::HEAVE),
                                 wave_frequency,
                                 wave_heading);

    motions.roll_velocity = -wave_amplitude * encounter_freq *
                            std::cos(encounter_freq * time) *
                            interpolateRAO(
                                rao.frequencies_rad_s,
                                rao.headings_deg,
                                rao.amplitudes.at(MotionType::ROLL),
                                wave_frequency,
                                wave_heading);

    motions.pitch_velocity = -wave_amplitude * encounter_freq *
                             std::cos(encounter_freq * time) *
                             interpolateRAO(
                                 rao.frequencies_rad_s,
                                 rao.headings_deg,
                                 rao.amplitudes.at(MotionType::PITCH),
                                 wave_frequency,
                                 wave_heading);

    // Compute accelerations (simplified as second harmonic derivatives)
    double omega2 = encounter_freq * encounter_freq;

    motions.heave_acceleration = -wave_amplitude * omega2 *
                                 std::sin(encounter_freq * time) *
                                 interpolateRAO(
                                     rao.frequencies_rad_s,
                                     rao.headings_deg,
                                     rao.amplitudes.at(MotionType::HEAVE),
                                     wave_frequency,
                                     wave_heading);

    motions.roll_acceleration = -wave_amplitude * omega2 *
                                std::sin(encounter_freq * time) *
                                interpolateRAO(
                                    rao.frequencies_rad_s,
                                    rao.headings_deg,
                                    rao.amplitudes.at(MotionType::ROLL),
                                    wave_frequency,
                                    wave_heading);

    motions.pitch_acceleration = -wave_amplitude * omega2 *
                                 std::sin(encounter_freq * time) *
                                 interpolateRAO(
                                     rao.frequencies_rad_s,
                                     rao.headings_deg,
                                     rao.amplitudes.at(MotionType::PITCH),
                                     wave_frequency,
                                     wave_heading);

    return motions;
}

LocationAcceleration SeakeepingModel::computeLocationAcceleration(
    const ShipMotions& motions,
    const Eigen::Vector3d& position) const
{
    LocationAcceleration accel;

    // Linear accelerations from ship motions
    Eigen::Vector3d linear_accel(
        motions.surge_acceleration,
        motions.sway_acceleration,
        motions.heave_acceleration);

    // Angular velocities and accelerations
    Eigen::Vector3d omega(
        motions.roll_velocity,
        motions.pitch_velocity,
        motions.yaw_velocity);
    Eigen::Vector3d alpha(
        motions.roll_acceleration,
        motions.pitch_acceleration,
        motions.yaw_acceleration);

    // Acceleration at position due to rotation
    // a = a_center + α × r + ω × (ω × r)
    Eigen::Vector3d centripetal = omega.cross(omega.cross(position));
    Eigen::Vector3d tangential = alpha.cross(position);

    accel.linear_accel = linear_accel + centripetal + tangential;
    accel.angular_accel = alpha;
    accel.total_magnitude = accel.linear_accel.norm();

    return accel;
}

double SeakeepingModel::computeMSI(
    const ShipMotions& motions,
    double exposure_time_hours) const
{
    // ISO 2631-1 based Motion Sickness Incidence
    // Simplified model using vertical acceleration

    // RMS vertical acceleration (m/s²)
    double az_rms = std::abs(motions.heave_acceleration) / std::sqrt(2.0);

    // Motion Sickness Dose Value (MSDV)
    // MSDV = az_rms * sqrt(T), where T is in seconds
    double T_seconds = exposure_time_hours * 3600.0;
    double msdv = az_rms * std::sqrt(T_seconds);

    // MSI percentage from MSDV (empirical formula)
    // MSI ≈ Φ((ln(MSDV) - 2.128) / 0.4)
    // where Φ is the cumulative normal distribution

    if (msdv < 0.01) {
        return 0.0;
    }

    double z = (std::log(msdv) - 2.128) / 0.4;

    // Approximation of cumulative normal distribution
    double msi = 50.0 * (1.0 + std::erf(z / std::sqrt(2.0)));

    // Clamp to 0-100%
    return std::max(0.0, std::min(100.0, msi));
}

OperabilityStatus SeakeepingModel::assessOperability(
    const ShipMotions& motions,
    const OperabilityCriteria& criteria,
    int sea_state,
    double wind_speed) const
{
    OperabilityStatus status;
    status.is_operable = true;
    status.overall_score = 1.0;

    // Check sea state limit
    if (sea_state > criteria.max_sea_state) {
        status.is_operable = false;
        status.limiting_factors.push_back("Sea state too high");
        status.overall_score *= 0.5;
    }

    // Check wind speed
    if (wind_speed > criteria.max_wind_speed_m_s) {
        status.is_operable = false;
        status.limiting_factors.push_back("Wind speed too high");
        status.overall_score *= 0.7;
    }

    // Check roll angle
    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;
    if (roll_deg > criteria.max_roll_deg) {
        status.is_operable = false;
        status.limiting_factors.push_back("Excessive roll angle");
        status.overall_score *= (criteria.max_roll_deg / roll_deg);
    }

    // Check pitch angle
    double pitch_deg = std::abs(motions.pitch) * 180.0 / M_PI;
    if (pitch_deg > criteria.max_pitch_deg) {
        status.is_operable = false;
        status.limiting_factors.push_back("Excessive pitch angle");
        status.overall_score *= (criteria.max_pitch_deg / pitch_deg);
    }

    // Check heave
    if (std::abs(motions.heave) > criteria.max_heave_m) {
        status.is_operable = false;
        status.limiting_factors.push_back("Excessive heave");
        status.overall_score *=
            (criteria.max_heave_m / std::abs(motions.heave));
    }

    // Check accelerations
    double g = 9.81;  // m/s²

    double vert_accel_g = std::abs(motions.heave_acceleration) / g;
    if (vert_accel_g > criteria.max_vertical_accel_g) {
        status.is_operable = false;
        status.limiting_factors.push_back("Excessive vertical acceleration");
        status.overall_score *= (criteria.max_vertical_accel_g / vert_accel_g);
    }

    // Check MSI
    status.msi_percent = computeMSI(motions, 2.0);  // 2 hour exposure
    if (status.msi_percent > criteria.max_msi_percent) {
        status.is_operable = false;
        status.limiting_factors.push_back("Excessive motion sickness risk");
        status.overall_score *= (criteria.max_msi_percent / status.msi_percent);
    }

    // Clamp overall score
    status.overall_score = std::max(0.0, std::min(1.0, status.overall_score));

    return status;
}

RAOData SeakeepingModel::createStandardRAO(const std::string& vessel_class)
{
    RAOData rao;

    // Standard frequency range (0.2 to 2.0 rad/s)
    int num_freq = 10;
    for (int i = 0; i < num_freq; ++i) {
        double freq = 0.2 + (i / double(num_freq - 1)) * 1.8;
        rao.frequencies_rad_s.push_back(freq);
    }

    // Standard heading range (0 to 180 deg, 0=head seas, 180=following seas)
    int num_heading = 7;
    for (int i = 0; i < num_heading; ++i) {
        double heading = i * 30.0;
        rao.headings_deg.push_back(heading);
    }

    // Initialize amplitude and phase matrices
    for (int m = 0; m <= 5; ++m) {
        MotionType motion = static_cast<MotionType>(m);
        rao.amplitudes[motion].resize(
            num_freq,
            std::vector<double>(num_heading, 0.0));
        rao.phases[motion].resize(
            num_freq,
            std::vector<double>(num_heading, 0.0));
    }

    // Parametric RAO based on vessel class
    double length = 100.0;  // Default length (m)
    double beam = 14.0;     // Default beam (m)
    double draft = 4.5;     // Default draft (m)

    if (vessel_class == "frigate") {
        length = 125.0;
        beam = 16.0;
        draft = 5.0;
    } else if (vessel_class == "corvette") {
        length = 90.0;
        beam = 12.0;
        draft = 4.0;
    } else if (vessel_class == "patrol_boat") {
        length = 50.0;
        beam = 8.0;
        draft = 2.5;
    }

    (void)beam;   // Could be used for roll RAO amplitude scaling
    (void)draft;  // Could be used for heave natural period

    // Simplified RAO model (empirical formulas)
    for (size_t i = 0; i < rao.frequencies_rad_s.size(); ++i) {
        double freq = rao.frequencies_rad_s[i];
        double omega_n = std::sqrt(9.81 / length);  // Natural frequency

        for (size_t j = 0; j < rao.headings_deg.size(); ++j) {
            double heading = rao.headings_deg[j];
            double heading_rad = heading * M_PI / 180.0;

            // Heave RAO (resonance around natural frequency)
            double heave_rao =
                1.0 /
                std::sqrt(
                    std::pow(1.0 - std::pow(freq / omega_n, 2.0), 2.0) + 0.1);
            heave_rao = std::min(heave_rao, 2.0);  // Cap at 2.0
            rao.amplitudes[MotionType::HEAVE][i][j] = heave_rao;

            // Roll RAO (maximum in beam seas)
            double roll_rao =
                1.5 * std::sin(heading_rad) /
                std::sqrt(
                    std::pow(1.0 - std::pow(freq / omega_n, 2.0), 2.0) + 0.2);
            roll_rao = std::min(roll_rao, 3.0) * (M_PI / 180.0);  // rad/m
            rao.amplitudes[MotionType::ROLL][i][j] = roll_rao;

            // Pitch RAO (maximum in head/following seas)
            double pitch_rao =
                1.0 * std::cos(heading_rad) /
                std::sqrt(
                    std::pow(1.0 - std::pow(freq / omega_n, 2.0), 2.0) + 0.15);
            pitch_rao = std::min(pitch_rao, 2.0) * (M_PI / 180.0);  // rad/m
            rao.amplitudes[MotionType::PITCH][i][j] = pitch_rao;

            // Surge (small, increases with frequency)
            rao.amplitudes[MotionType::SURGE][i][j] =
                0.5 * freq / omega_n * std::cos(heading_rad);

            // Sway (small, maximum in beam seas)
            rao.amplitudes[MotionType::SWAY][i][j] =
                0.5 * freq / omega_n * std::sin(heading_rad);

            // Yaw (small)
            rao.amplitudes[MotionType::YAW][i][j] =
                0.2 * freq / omega_n * (M_PI / 180.0);

            // Phases (simplified, all around 90 degrees at resonance)
            for (int m = 0; m <= 5; ++m) {
                MotionType motion = static_cast<MotionType>(m);
                rao.phases[motion][i][j] =
                    M_PI / 2.0;  // 90 degrees (in radians)
            }
        }
    }

    return rao;
}

OperabilityCriteria SeakeepingModel::getStandardCriteria(
    const std::string& vessel_type)
{
    OperabilityCriteria criteria;

    if (vessel_type == "frigate") {
        criteria.max_roll_deg = 20.0;
        criteria.max_pitch_deg = 10.0;
        criteria.max_heave_m = 4.0;
        criteria.max_sea_state = 6;
        criteria.max_wind_speed_m_s = 25.0;
        criteria.max_msi_percent = 20.0;
    } else if (vessel_type == "corvette") {
        criteria.max_roll_deg = 18.0;
        criteria.max_pitch_deg = 8.0;
        criteria.max_heave_m = 3.5;
        criteria.max_sea_state = 5;
        criteria.max_wind_speed_m_s = 22.0;
        criteria.max_msi_percent = 25.0;
    } else if (vessel_type == "patrol_boat") {
        criteria.max_roll_deg = 15.0;
        criteria.max_pitch_deg = 7.0;
        criteria.max_heave_m = 2.5;
        criteria.max_sea_state = 4;
        criteria.max_wind_speed_m_s = 18.0;
        criteria.max_msi_percent = 30.0;
    }

    return criteria;
}

double SeakeepingModel::computeSingleMotion(
    const RAOData& rao,
    MotionType motion_type,
    double wave_amplitude,
    double wave_frequency,
    double wave_heading,
    double time,
    double ship_speed) const
{
    // Get RAO amplitude for this motion
    double rao_amp = interpolateRAO(
        rao.frequencies_rad_s,
        rao.headings_deg,
        rao.amplitudes.at(motion_type),
        wave_frequency,
        wave_heading);

    double rao_phase = interpolateRAO(
        rao.frequencies_rad_s,
        rao.headings_deg,
        rao.phases.at(motion_type),
        wave_frequency,
        wave_heading);

    // Encounter frequency
    double encounter_freq =
        computeEncounterFrequency(wave_frequency, wave_heading, ship_speed);

    // Motion = RAO_amplitude * wave_amplitude * sin(omega_e * t + phase)
    double motion =
        rao_amp * wave_amplitude * std::sin(encounter_freq * time + rao_phase);

    return motion;
}

double SeakeepingModel::interpolateRAO(
    const std::vector<double>& frequencies,
    const std::vector<double>& headings,
    const std::vector<std::vector<double>>& values,
    double freq,
    double heading) const
{
    // Bilinear interpolation

    // Find frequency indices
    size_t i_freq = 0;
    for (size_t i = 1; i < frequencies.size(); ++i) {
        if (freq <= frequencies[i]) {
            i_freq = i - 1;
            break;
        }
    }
    i_freq = std::min(i_freq, frequencies.size() - 2);

    // Find heading indices
    size_t i_head = 0;
    for (size_t i = 1; i < headings.size(); ++i) {
        if (heading <= headings[i]) {
            i_head = i - 1;
            break;
        }
    }
    i_head = std::min(i_head, headings.size() - 2);

    // Interpolation weights
    double t_freq = (freq - frequencies[i_freq]) /
                    (frequencies[i_freq + 1] - frequencies[i_freq]);
    double t_head = (heading - headings[i_head]) /
                    (headings[i_head + 1] - headings[i_head]);

    t_freq = std::max(0.0, std::min(1.0, t_freq));
    t_head = std::max(0.0, std::min(1.0, t_head));

    // Bilinear interpolation
    double v00 = values[i_freq][i_head];
    double v10 = values[i_freq + 1][i_head];
    double v01 = values[i_freq][i_head + 1];
    double v11 = values[i_freq + 1][i_head + 1];

    double v0 = v00 * (1.0 - t_freq) + v10 * t_freq;
    double v1 = v01 * (1.0 - t_freq) + v11 * t_freq;

    return v0 * (1.0 - t_head) + v1 * t_head;
}

double SeakeepingModel::computeEncounterFrequency(
    double wave_frequency,
    double wave_heading,
    double ship_speed) const
{
    // Encounter frequency = wave_frequency + (wave_frequency^2 * V *
    // cos(heading) / g) Simplified: omega_e = omega * (1 + (V/c) *
    // cos(heading)) where c = g/omega (wave celerity)

    double g = 9.81;
    double wave_celerity = g / wave_frequency;
    double heading_rad = wave_heading * M_PI / 180.0;

    double encounter_freq =
        wave_frequency *
        (1.0 + (ship_speed / wave_celerity) * std::cos(heading_rad));

    return encounter_freq;
}

}  // namespace environment
}  // namespace lotusim
