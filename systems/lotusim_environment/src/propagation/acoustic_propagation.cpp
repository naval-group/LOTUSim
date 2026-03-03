/**
 * @file acoustic_propagation.cpp
 * @brief Advanced underwater acoustic propagation implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/propagation/acoustic_propagation.hpp"

#include <algorithm>
#include <cmath>

#include "lotusim_environment/ocean/ocean_model.hpp"

namespace lotusim {
namespace environment {

// ============================================================================
// AcousticPropagationModel Implementation
// ============================================================================

AcousticPropagationModel::AcousticPropagationModel(
    const ThermalProfile* thermal_profile)
    : thermal_profile_(thermal_profile)
{
}

RayTracingResult AcousticPropagationModel::traceRays(
    const Eigen::Vector3d& source_pos,
    const Eigen::Vector3d& receiver_pos,
    double frequency_hz,
    double water_depth_m) const
{
    (void)frequency_hz;  // Could be used for frequency-dependent absorption
    RayTracingResult result;

    // Horizontal distance
    Eigen::Vector2d horiz_source(source_pos.x(), source_pos.y());
    Eigen::Vector2d horiz_receiver(receiver_pos.x(), receiver_pos.y());
    double horiz_range = (horiz_receiver - horiz_source).norm();

    // Trace rays at different launch angles
    std::vector<double> launch_angles;
    for (int i = 0; i < NUM_RAYS; ++i) {
        double angle = -90.0 + (i / double(NUM_RAYS - 1)) * 180.0;
        launch_angles.push_back(angle);
    }

    // Find eigenrays (rays that pass near the receiver)
    double tolerance_m = 50.0;  // Receiver capture radius

    for (double angle : launch_angles) {
        AcousticRay ray =
            traceSingleRay(source_pos, angle, horiz_range * 1.5, water_depth_m);

        // Check if ray passes near receiver
        for (size_t i = 0; i < ray.waypoints.size(); ++i) {
            double dist_to_receiver = (ray.waypoints[i] - receiver_pos).norm();
            if (dist_to_receiver < tolerance_m) {
                Eigenray eigenray;
                eigenray.ray = ray;
                eigenray.launch_angle_deg = angle;
                result.eigenrays.push_back(eigenray);
                break;
            }
        }
    }

    // Compute total transmission loss
    if (result.eigenrays.empty()) {
        // No eigenrays found - shadow zone
        result.is_shadow_zone = true;
        result.transmission_loss_dB = 200.0;  // Very high loss
    } else {
        // Find eigenray with minimum loss
        double min_loss = 1e9;
        for (const auto& eigenray : result.eigenrays) {
            if (eigenray.ray.path_loss_dB < min_loss) {
                min_loss = eigenray.ray.path_loss_dB;
            }
        }
        result.transmission_loss_dB = min_loss;
    }

    return result;
}

double AcousticPropagationModel::computeTransmissionLoss(
    const Eigen::Vector3d& source_pos,
    const Eigen::Vector3d& receiver_pos,
    double frequency_hz) const
{
    double distance = (receiver_pos - source_pos).norm();

    // Simple model: spreading + absorption
    double water_depth_m = 1000.0;  // Default deep water
    double spreading_loss = computeSpreadingLoss(distance, water_depth_m);
    double absorption_loss = computeAbsorptionLoss(distance, frequency_hz);

    return spreading_loss + absorption_loss;
}

bool AcousticPropagationModel::isInSofarChannel(double depth_m) const
{
    if (!thermal_profile_)
        return false;

    SofarChannelModel sofar(thermal_profile_);
    sofar.computeChannelParameters();

    return sofar.isInChannel(depth_m);
}

double AcousticPropagationModel::getSofarAxisDepth() const
{
    if (!thermal_profile_)
        return 1000.0;

    SofarChannelModel sofar(thermal_profile_);
    sofar.computeChannelParameters();

    return sofar.getAxisDepth();
}

void AcousticPropagationModel::setBottomType(const std::string& type)
{
    bottom_type_ = type;

    // Set reflection loss coefficient based on bottom type
    if (type == "sand") {
        bottom_loss_coeff_ = 0.5;  // Moderate loss
    } else if (type == "mud") {
        bottom_loss_coeff_ = 0.7;  // High loss
    } else if (type == "rock") {
        bottom_loss_coeff_ = 0.2;  // Low loss (hard bottom)
    } else if (type == "gravel") {
        bottom_loss_coeff_ = 0.4;  // Moderate-low loss
    }
}

double AcousticPropagationModel::getSoundSpeed(double depth_m) const
{
    if (thermal_profile_) {
        // Use thermal profile
        return thermal_profile_->sound_speed_mps.empty()
                   ? 1500.0
                   : ThermalProfileModel::computeSoundSpeed(
                         thermal_profile_->temperature_C.front(),
                         35.0,
                         depth_m);
    } else {
        // Default sound speed profile
        // Minimum at ~1000m (SOFAR channel)
        if (depth_m < 1000.0) {
            return 1540.0 - 0.05 * depth_m;  // Decreasing to SOFAR axis
        } else {
            return 1490.0 +
                   0.016 * (depth_m - 1000.0);  // Increasing below SOFAR
        }
    }
}

AcousticRay AcousticPropagationModel::traceSingleRay(
    const Eigen::Vector3d& source_pos,
    double launch_angle_deg,
    double max_range_m,
    double water_depth_m) const
{
    AcousticRay ray;

    // Initial position and direction
    Eigen::Vector3d pos = source_pos;
    double angle_deg = launch_angle_deg;

    // Ray step
    double step = RAY_STEP_M;

    // Trace ray
    int max_steps = static_cast<int>(max_range_m / step);
    for (int i = 0; i < max_steps; ++i) {
        ray.waypoints.push_back(pos);

        double depth = -pos.z();  // Convert NED z to depth
        double c = getSoundSpeed(depth);
        ray.sound_speeds.push_back(c);

        // Compute next position using current angle
        double angle_rad = angle_deg * M_PI / 180.0;
        Eigen::Vector3d dir(
            std::cos(angle_rad),  // Horizontal component
            0.0,                  // Assume 2D propagation in x-z plane
            -std::sin(
                angle_rad)  // Vertical component (down is positive in depth)
        );

        Eigen::Vector3d next_pos = pos + dir * step;

        // Check boundaries
        double next_depth = -next_pos.z();

        // Surface reflection
        if (next_depth < 0.0) {
            next_pos.z() = 0.0;      // At surface
            angle_deg = -angle_deg;  // Reflect angle
            ray.path_loss_dB += computeSurfaceReflectionLoss();
            ray.num_reflections++;
        }

        // Bottom reflection
        if (next_depth > water_depth_m) {
            next_pos.z() = -water_depth_m;  // At bottom
            angle_deg = -angle_deg;         // Reflect angle
            ray.path_loss_dB +=
                computeBottomReflectionLoss(std::abs(angle_deg));
            ray.num_reflections++;
        }

        // Apply Snell's law for refraction
        double c_next = getSoundSpeed(next_depth);
        angle_deg = applySnellsLaw(angle_deg, c, c_next);

        // Update position
        pos = next_pos;

        // Accumulate path loss
        double segment_loss =
            computeSpreadingLoss(step, water_depth_m) +
            computeAbsorptionLoss(step, 5000.0);  // Assume 5 kHz
        ray.path_loss_dB += segment_loss;
        ray.travel_time_s += step / c;

        // Stop if ray escapes upward or downward
        if (depth < -100.0 || depth > water_depth_m + 100.0) {
            break;
        }
    }

    // Check if ray stayed in SOFAR channel
    bool all_in_sofar = true;
    for (const auto& wp : ray.waypoints) {
        if (!isInSofarChannel(-wp.z())) {
            all_in_sofar = false;
            break;
        }
    }
    ray.is_sofar_channel = all_in_sofar;

    return ray;
}

double AcousticPropagationModel::computeSpreadingLoss(
    double distance_m,
    double water_depth_m) const
{
    // Spherical spreading: 20·log10(r)
    // Cylindrical spreading: 10·log10(r)
    // Transition at r ≈ water_depth

    if (distance_m < 1.0)
        distance_m = 1.0;

    if (distance_m < water_depth_m) {
        // Spherical spreading
        return 20.0 * std::log10(distance_m);
    } else {
        // Spherical to water depth, then cylindrical
        double spherical_part = 20.0 * std::log10(water_depth_m);
        double cylindrical_part = 10.0 * std::log10(distance_m / water_depth_m);
        return spherical_part + cylindrical_part;
    }
}

double AcousticPropagationModel::computeAbsorptionLoss(
    double distance_m,
    double frequency_hz) const
{
    // Use Francois-Garrison absorption model
    double alpha_dB_per_km = AbsorptionModel::computeAbsorption(
        frequency_hz,
        15.0,   // Temperature (°C)
        35.0,   // Salinity (ppt)
        100.0,  // Depth (m)
        8.0);   // pH

    double distance_km = distance_m / 1000.0;
    return alpha_dB_per_km * distance_km;
}

double AcousticPropagationModel::computeSurfaceReflectionLoss() const
{
    // Surface reflection loss depends on sea state
    // Calm sea: ~1 dB, Rough sea: ~5-10 dB

    double base_loss_dB = 1.0;
    double roughness_loss_dB = 5.0 * (sea_surface_roughness_m_ / 3.0);

    return base_loss_dB + roughness_loss_dB;
}

double AcousticPropagationModel::computeBottomReflectionLoss(
    double grazing_angle_deg) const
{
    // Bottom loss depends on grazing angle and bottom type
    // Higher grazing angles = more loss

    double base_loss_dB = bottom_loss_coeff_ * 5.0;
    double angle_factor = std::sin(grazing_angle_deg * M_PI / 180.0);
    double angle_loss_dB = bottom_loss_coeff_ * 10.0 * angle_factor;

    return base_loss_dB + angle_loss_dB;
}

double AcousticPropagationModel::applySnellsLaw(
    double incident_angle_deg,
    double c1,
    double c2) const
{
    // Snell's law: c1/cos(θ1) = c2/cos(θ2)
    // θ measured from horizontal

    if (std::abs(c1 - c2) < 0.1) {
        return incident_angle_deg;  // No refraction
    }

    double theta1_rad = incident_angle_deg * M_PI / 180.0;
    double cos_theta1 = std::cos(theta1_rad);

    if (std::abs(cos_theta1) < 0.01) {
        return incident_angle_deg;  // Nearly horizontal
    }

    double cos_theta2 = (c2 / c1) * cos_theta1;

    // Clamp to valid range
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));

    double theta2_rad = std::acos(cos_theta2);
    double theta2_deg = theta2_rad * 180.0 / M_PI;

    // Preserve sign
    if (incident_angle_deg < 0.0) {
        theta2_deg = -theta2_deg;
    }

    return theta2_deg;
}

// ============================================================================
// SofarChannelModel Implementation
// ============================================================================

SofarChannelModel::SofarChannelModel(const ThermalProfile* thermal_profile)
    : thermal_profile_(thermal_profile)
{
}

void SofarChannelModel::computeChannelParameters()
{
    if (!thermal_profile_ || thermal_profile_->sound_speed_mps.empty()) {
        // Use default values
        return;
    }

    // Find depth of minimum sound speed
    double min_c = 1e9;
    size_t min_idx = 0;

    for (size_t i = 0; i < thermal_profile_->sound_speed_mps.size(); ++i) {
        if (thermal_profile_->sound_speed_mps[i] < min_c) {
            min_c = thermal_profile_->sound_speed_mps[i];
            min_idx = i;
        }
    }

    axis_depth_m_ = thermal_profile_->depth_m[min_idx];
    min_sound_speed_mps_ = min_c;

    // Channel half-width: find where sound speed increases by 2 m/s
    double threshold_c = min_c + 2.0;

    // Search upward
    double upper_depth = axis_depth_m_;
    for (int i = min_idx; i >= 0; --i) {
        if (thermal_profile_->sound_speed_mps[i] > threshold_c) {
            upper_depth = thermal_profile_->depth_m[i];
            break;
        }
    }

    // Search downward
    double lower_depth = axis_depth_m_;
    for (size_t i = min_idx; i < thermal_profile_->sound_speed_mps.size();
         ++i) {
        if (thermal_profile_->sound_speed_mps[i] > threshold_c) {
            lower_depth = thermal_profile_->depth_m[i];
            break;
        }
    }

    channel_half_width_m_ =
        std::max(axis_depth_m_ - upper_depth, lower_depth - axis_depth_m_);
}

double SofarChannelModel::computeSofarLoss(
    double distance_m,
    double frequency_hz) const
{
    // SOFAR channel propagation has low loss due to waveguide effect
    // Use cylindrical spreading over most of the path

    if (distance_m < 1.0)
        distance_m = 1.0;

    // Cylindrical spreading: 10·log10(r)
    double spreading_loss = 10.0 * std::log10(distance_m);

    // Absorption
    double alpha_dB_per_km = AbsorptionModel::computeAbsorption(
        frequency_hz,
        10.0,  // Temperature at depth (°C)
        35.0,  // Salinity (ppt)
        axis_depth_m_,
        8.0);

    double distance_km = distance_m / 1000.0;
    double absorption_loss = alpha_dB_per_km * distance_km;

    return spreading_loss + absorption_loss;
}

// ============================================================================
// AbsorptionModel Implementation (Francois-Garrison)
// ============================================================================

double AbsorptionModel::computeAbsorption(
    double frequency_hz,
    double temperature_C,
    double salinity_ppt,
    double depth_m,
    double pH)
{
    double alpha_boric = boricAcidContribution(
        frequency_hz,
        temperature_C,
        salinity_ppt,
        depth_m,
        pH);
    double alpha_MgSO4 = magnesiumSulfateContribution(
        frequency_hz,
        temperature_C,
        salinity_ppt,
        depth_m);
    double alpha_water = pureWaterContribution(
        frequency_hz,
        temperature_C,
        salinity_ppt,
        depth_m);

    return alpha_boric + alpha_MgSO4 + alpha_water;
}

double AbsorptionModel::boricAcidContribution(
    double frequency_hz,
    double temperature_C,
    double salinity_ppt,
    double depth_m,
    double pH)
{
    (void)depth_m;  // Could be used for pressure effects
    (void)pH;       // Could be used for pH-dependent absorption

    double f_kHz = frequency_hz / 1000.0;
    double T = temperature_C;
    double S = salinity_ppt;

    // Boric acid relaxation frequency
    double f1 = 0.78 * std::sqrt(S / 35.0) * std::exp(T / 26.0);

    // Boric acid absorption
    double A1 = 0.106 * (f1 * f_kHz * f_kHz) / (f1 * f1 + f_kHz * f_kHz);

    return A1;
}

double AbsorptionModel::magnesiumSulfateContribution(
    double frequency_hz,
    double temperature_C,
    double salinity_ppt,
    double depth_m)
{
    (void)depth_m;  // Could be used for pressure effects

    double f_kHz = frequency_hz / 1000.0;
    double T = temperature_C;
    double S = salinity_ppt;

    // MgSO4 relaxation frequency
    double f2 = 42.0 * std::exp(T / 17.0);

    // MgSO4 absorption
    double A2 =
        0.52 * (S / 35.0) * (f2 * f_kHz * f_kHz) / (f2 * f2 + f_kHz * f_kHz);

    return A2;
}

double AbsorptionModel::pureWaterContribution(
    double frequency_hz,
    double temperature_C,
    double salinity_ppt,
    double depth_m)
{
    (void)salinity_ppt;  // Pure water model doesn't depend on salinity

    double f_kHz = frequency_hz / 1000.0;
    double T = temperature_C;

    // Pure water absorption
    double A3 =
        0.00049 * f_kHz * f_kHz * std::exp(-(T / 27.0 + depth_m / 17000.0));

    return A3;
}

}  // namespace environment
}  // namespace lotusim
