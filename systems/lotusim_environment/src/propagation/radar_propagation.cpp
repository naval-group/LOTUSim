/**
 * @file radar_propagation.cpp
 * @brief Advanced radar propagation implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/propagation/radar_propagation.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim {
namespace environment {

// ============================================================================
// RadarPropagationModel Implementation
// ============================================================================

RadarPropagationModel::RadarPropagationModel()
{
    refractivity_profile_ = createStandardAtmosphere();
}

std::vector<PropagationPath> RadarPropagationModel::computePaths(
    const Eigen::Vector3d& tx_pos,
    const Eigen::Vector3d& rx_pos,
    double frequency_hz) const
{
    std::vector<PropagationPath> paths;

    // Always compute direct path
    PropagationPath direct_path =
        computeDirectPath(tx_pos, rx_pos, frequency_hz);
    paths.push_back(direct_path);

    // Compute surface reflection path (if both above water)
    if (tx_pos.z() <= 0.0 && rx_pos.z() <= 0.0) {
        PropagationPath reflection_path =
            computeReflectionPath(tx_pos, rx_pos, frequency_hz);
        paths.push_back(reflection_path);
    }

    // Compute ducted path if duct is present
    if (has_duct_) {
        PropagationPath ducted_path =
            computeDuctedPath(tx_pos, rx_pos, frequency_hz);
        if (ducted_path.segments.size() > 0) {
            paths.push_back(ducted_path);
        }
    }

    return paths;
}

double RadarPropagationModel::getRadarHorizon(
    double tx_altitude_m,
    double rx_altitude_m) const
{
    // Radar horizon with effective Earth radius
    double k = STANDARD_K_FACTOR;
    double Re = EARTH_RADIUS_M * k;

    // Geometric horizon distance
    double d1 = std::sqrt(2.0 * Re * std::abs(tx_altitude_m));
    double d2 = std::sqrt(2.0 * Re * std::abs(rx_altitude_m));

    return d1 + d2;
}

bool RadarPropagationModel::isInDuct(double altitude_m) const
{
    if (!has_duct_)
        return false;

    return (
        altitude_m >= duct_.base_altitude_m &&
        altitude_m <= duct_.top_altitude_m);
}

void RadarPropagationModel::setAtmosphericConditions(
    double temperature_C,
    double humidity_percent,
    double pressure_hPa)
{
    temperature_C_ = temperature_C;
    humidity_percent_ = humidity_percent;
    pressure_hPa_ = pressure_hPa;
}

RefractivityProfile RadarPropagationModel::createStandardAtmosphere()
{
    RefractivityProfile profile;

    // Standard atmosphere (decreasing refractivity with altitude)
    profile.altitude_m = {0, 100, 500, 1000, 2000, 5000, 10000};

    // N-units: N = (n - 1) × 10^6
    // Standard atmosphere: N ≈ 315 at sea level, decreasing with altitude
    profile.N_units = {315.0, 310.0, 295.0, 280.0, 250.0, 180.0, 100.0};

    return profile;
}

RefractivityProfile RadarPropagationModel::createEvaporationDuct(
    double duct_height_m)
{
    RefractivityProfile profile;

    // Evaporation duct profile (common over ocean)
    int num_points = 20;
    profile.altitude_m.resize(num_points);
    profile.N_units.resize(num_points);

    for (int i = 0; i < num_points; ++i) {
        double h = (i / double(num_points - 1)) * duct_height_m * 3.0;
        profile.altitude_m[i] = h;

        if (h < duct_height_m) {
            // Inside duct: modified refractivity gradient
            profile.N_units[i] = 315.0 - 0.125 * h * h / duct_height_m;
        } else {
            // Above duct: standard atmosphere
            profile.N_units[i] = 315.0 - 0.118 * h;
        }
    }

    return profile;
}

PropagationPath RadarPropagationModel::computeDirectPath(
    const Eigen::Vector3d& tx_pos,
    const Eigen::Vector3d& rx_pos,
    double frequency_hz) const
{
    PropagationPath path;

    RaySegment segment;
    segment.start = tx_pos;
    segment.end = rx_pos;

    Eigen::Vector3d diff = rx_pos - tx_pos;
    double distance = diff.norm();

    // Free space path loss
    segment.path_loss_dB = computeFreeSpaceLoss(distance, frequency_hz);

    // Atmospheric absorption (simplified)
    double atm_absorption_dB_km = 0.01;  // Typical X-band
    segment.path_loss_dB += atm_absorption_dB_km * (distance / 1000.0);

    path.segments.push_back(segment);
    path.total_path_loss_dB = segment.path_loss_dB;
    path.total_distance_m = distance;
    path.is_ducted = false;
    path.num_reflections = 0;

    return path;
}

PropagationPath RadarPropagationModel::computeReflectionPath(
    const Eigen::Vector3d& tx_pos,
    const Eigen::Vector3d& rx_pos,
    double frequency_hz) const
{
    PropagationPath path;

    // Find reflection point on water surface (z = 0)
    // Simple model: midpoint horizontally at sea surface
    Eigen::Vector3d reflection_point = (tx_pos + rx_pos) / 2.0;
    reflection_point.z() = 0.0;

    // First segment: TX to reflection point
    RaySegment seg1;
    seg1.start = tx_pos;
    seg1.end = reflection_point;
    double d1 = (reflection_point - tx_pos).norm();
    seg1.path_loss_dB = computeFreeSpaceLoss(d1, frequency_hz);

    // Second segment: reflection point to RX
    RaySegment seg2;
    seg2.start = reflection_point;
    seg2.end = rx_pos;
    double d2 = (rx_pos - reflection_point).norm();
    seg2.path_loss_dB = computeFreeSpaceLoss(d2, frequency_hz);

    // Reflection loss (Fresnel reflection coefficient)
    // For grazing angles and sea water: ~3-6 dB
    double reflection_loss_dB = 4.0;

    // Grazing angle
    double grazing_angle = std::atan2(std::abs(tx_pos.z()), d1 / 2.0);

    // Divergence factor
    double divergence_factor = MultipathModel::computeDivergenceFactor(
        grazing_angle,
        d1,
        frequency_hz);
    double divergence_loss_dB = -10.0 * std::log10(divergence_factor);

    path.segments.push_back(seg1);
    path.segments.push_back(seg2);
    path.total_path_loss_dB = seg1.path_loss_dB + seg2.path_loss_dB +
                              reflection_loss_dB + divergence_loss_dB;
    path.total_distance_m = d1 + d2;
    path.is_ducted = false;
    path.num_reflections = 1;

    return path;
}

PropagationPath RadarPropagationModel::computeDuctedPath(
    const Eigen::Vector3d& tx_pos,
    const Eigen::Vector3d& rx_pos,
    double frequency_hz) const
{
    PropagationPath path;

    // Check if both TX and RX are within the duct
    bool tx_in_duct = isInDuct(-tx_pos.z());
    bool rx_in_duct = isInDuct(-rx_pos.z());

    if (!tx_in_duct && !rx_in_duct) {
        return path;  // Empty path
    }

    // Simplified ducted path model
    RaySegment segment;
    segment.start = tx_pos;
    segment.end = rx_pos;

    Eigen::Vector3d diff = rx_pos - tx_pos;
    double distance = diff.norm();

    // Ducted propagation has lower path loss than free space
    // Use modified refractive index
    double duct_advantage_dB = 20.0 * duct_.strength;  // 0-20 dB advantage

    segment.path_loss_dB =
        computeFreeSpaceLoss(distance, frequency_hz) - duct_advantage_dB;

    path.segments.push_back(segment);
    path.total_path_loss_dB = segment.path_loss_dB;
    path.total_distance_m = distance;
    path.is_ducted = true;
    path.num_reflections = 0;

    return path;
}

double RadarPropagationModel::computeFreeSpaceLoss(
    double distance_m,
    double frequency_hz) const
{
    // Friis transmission equation
    // FSPL (dB) = 20·log10(d) + 20·log10(f) + 20·log10(4π/c)
    //           = 20·log10(d) + 20·log10(f) - 147.55

    double c = 299792458.0;  // Speed of light (m/s)

    if (distance_m < 1.0)
        distance_m = 1.0;

    double fspl_dB = 20.0 * std::log10(distance_m) +
                     20.0 * std::log10(frequency_hz) +
                     20.0 * std::log10(4.0 * M_PI / c);

    return fspl_dB;
}

double RadarPropagationModel::getRefractivity(double altitude_m) const
{
    // Linear interpolation in refractivity profile
    const auto& alt = refractivity_profile_.altitude_m;
    const auto& N = refractivity_profile_.N_units;

    if (altitude_m <= alt.front())
        return N.front();
    if (altitude_m >= alt.back())
        return N.back();

    for (size_t i = 1; i < alt.size(); ++i) {
        if (altitude_m <= alt[i]) {
            double t = (altitude_m - alt[i - 1]) / (alt[i] - alt[i - 1]);
            return N[i - 1] + t * (N[i] - N[i - 1]);
        }
    }

    return N.back();
}

double RadarPropagationModel::getRefractiveGradient(double altitude_m) const
{
    // Compute dN/dh numerically
    double delta_h = 10.0;  // 10 m
    double N1 = getRefractivity(altitude_m);
    double N2 = getRefractivity(altitude_m + delta_h);
    return (N2 - N1) / delta_h;
}

double RadarPropagationModel::getEffectiveEarthRadius() const
{
    // k-factor depends on refractive gradient
    // Standard atmosphere: k = 4/3
    // Can be modified based on actual gradient
    return EARTH_RADIUS_M * STANDARD_K_FACTOR;
}

double RadarPropagationModel::computeFresnelClearance(
    const Eigen::Vector3d& tx_pos,
    const Eigen::Vector3d& rx_pos,
    double frequency_hz) const
{
    double c = 299792458.0;
    double wavelength = c / frequency_hz;
    double distance = (rx_pos - tx_pos).norm();

    // First Fresnel zone radius at midpoint
    double r1 = std::sqrt(wavelength * distance / 4.0);

    return r1;
}

// ============================================================================
// MultipathModel Implementation
// ============================================================================

double MultipathModel::computeMultipathFading(
    const std::vector<PropagationPath>& paths,
    double frequency_hz)
{
    if (paths.size() < 2) {
        return 0.0;  // No multipath
    }

    // Simplified model: combine paths vectorially
    // Assumes worst-case destructive interference
    double c = 299792458.0;
    double wavelength = c / frequency_hz;

    // Path difference
    double delta_path = paths[1].total_distance_m - paths[0].total_distance_m;
    double phase_diff = 2.0 * M_PI * delta_path / wavelength;

    // Amplitude ratio (linear)
    double a1 = std::pow(10.0, -paths[0].total_path_loss_dB / 20.0);
    double a2 = std::pow(10.0, -paths[1].total_path_loss_dB / 20.0);

    // Vectorial combination
    double E_real = a1 + a2 * std::cos(phase_diff);
    double E_imag = a2 * std::sin(phase_diff);
    double E_total = std::sqrt(E_real * E_real + E_imag * E_imag);

    // Single path amplitude
    double E_single = a1;

    // Fading (dB)
    double fading_dB = 20.0 * std::log10(E_total / E_single);

    return fading_dB;
}

double MultipathModel::getRayleighFadingMargin(int num_paths)
{
    // For Rayleigh fading, the margin for 99% availability
    // depends on the number of paths

    if (num_paths <= 1)
        return 0.0;

    // Typical values for maritime environment
    // 99% availability margin (dB)
    if (num_paths == 2)
        return 10.0;
    else if (num_paths == 3)
        return 15.0;
    else
        return 20.0;
}

double MultipathModel::computeDivergenceFactor(
    double grazing_angle_rad,
    double distance_m,
    double frequency_hz)
{
    (void)frequency_hz;  // Could be used for frequency-dependent effects

    // Divergence factor accounts for Earth curvature
    // For small grazing angles over spherical Earth

    double Re = RadarPropagationModel::EARTH_RADIUS_M;
    double psi = grazing_angle_rad;

    // Simplified divergence factor
    double D = 1.0 / (1.0 + (distance_m / Re) * std::tan(psi));

    // Clamp between 0 and 1
    return std::max(0.0, std::min(1.0, D));
}

}  // namespace environment
}  // namespace lotusim
