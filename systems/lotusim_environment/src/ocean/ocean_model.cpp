/**
 * @file ocean_model.cpp
 * @brief Ocean modeling implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/ocean/ocean_model.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim {
namespace environment {

// ============================================================================
// Ocean Currents (Sprint 9.2.1)
// ============================================================================

OceanCurrentModel::OceanCurrentModel(const CurrentParameters& params)
    : params_(params), elapsed_time_(0.0)
{
}

void OceanCurrentModel::update(double dt)
{
    elapsed_time_ += dt;
}

Eigen::Vector3d OceanCurrentModel::getCurrentVelocity(
    const Eigen::Vector3d& position,
    double depth) const
{
    (void)position;  // Could be used for spatial variation

    // Exponential decay with depth
    double depth_factor = std::exp(-params_.depth_decay_coefficient * depth);

    // Tidal component
    double tidal_freq = 2.0 * M_PI / (params_.tidal_period_h * 3600.0);
    double tidal_mult =
        1.0 + params_.tidal_amplitude * std::sin(tidal_freq * elapsed_time_);

    Eigen::Vector3d current =
        params_.surface_velocity * depth_factor * tidal_mult;

    return current;
}

// ============================================================================
// Thermal Profiles (Sprint 9.2.2)
// ============================================================================

ThermalProfileModel::ThermalProfileModel()
{
    profile_ = createDefaultProfile();
}

void ThermalProfileModel::loadProfile(
    const std::string& region,
    const std::string& season)
{
    if (region == "Mediterranean" && season == "summer") {
        profile_ = createMediterraneanSummer();
    } else if (region == "Atlantic" && season == "winter") {
        profile_ = createAtlanticWinter();
    } else {
        profile_ = createDefaultProfile();
    }
}

void ThermalProfileModel::setProfile(const ThermalProfile& profile)
{
    profile_ = profile;
}

double ThermalProfileModel::getTemperature(double depth) const
{
    return interpolate(profile_.depth_m, profile_.temperature_C, depth);
}

double ThermalProfileModel::getSoundSpeed(double depth) const
{
    return interpolate(profile_.depth_m, profile_.sound_speed_mps, depth);
}

double ThermalProfileModel::getThermoclineDepth() const
{
    // Find depth of maximum temperature gradient
    double max_gradient = 0.0;
    double thermocline_depth = 50.0;

    for (size_t i = 1; i < profile_.depth_m.size(); ++i) {
        double dT = profile_.temperature_C[i - 1] - profile_.temperature_C[i];
        double dz = profile_.depth_m[i] - profile_.depth_m[i - 1];
        double gradient = std::abs(dT / dz);

        if (gradient > max_gradient) {
            max_gradient = gradient;
            thermocline_depth = profile_.depth_m[i];
        }
    }

    return thermocline_depth;
}

ThermalProfile ThermalProfileModel::createDefaultProfile()
{
    ThermalProfile profile;

    // Simple 3-layer model
    profile.depth_m = {0, 50, 100, 200, 500, 1000, 2000};
    profile.temperature_C = {20.0, 18.0, 12.0, 8.0, 6.0, 5.0, 4.0};

    // Compute sound speed using Mackenzie equation
    for (size_t i = 0; i < profile.depth_m.size(); ++i) {
        double c = computeSoundSpeed(
            profile.temperature_C[i],
            35.0,
            profile.depth_m[i]);
        profile.sound_speed_mps.push_back(c);
    }

    return profile;
}

ThermalProfile ThermalProfileModel::createMediterraneanSummer()
{
    ThermalProfile profile;

    profile.depth_m = {0, 20, 40, 80, 150, 300, 1000};
    profile.temperature_C = {26.0, 24.0, 20.0, 15.0, 14.0, 13.5, 13.0};

    for (size_t i = 0; i < profile.depth_m.size(); ++i) {
        double c = computeSoundSpeed(
            profile.temperature_C[i],
            38.0,
            profile.depth_m[i]);
        profile.sound_speed_mps.push_back(c);
    }

    return profile;
}

ThermalProfile ThermalProfileModel::createAtlanticWinter()
{
    ThermalProfile profile;

    profile.depth_m = {0, 50, 100, 200, 500, 1000, 2000};
    profile.temperature_C = {10.0, 9.5, 9.0, 8.0, 6.0, 4.5, 3.5};

    for (size_t i = 0; i < profile.depth_m.size(); ++i) {
        double c = computeSoundSpeed(
            profile.temperature_C[i],
            35.0,
            profile.depth_m[i]);
        profile.sound_speed_mps.push_back(c);
    }

    return profile;
}

double ThermalProfileModel::interpolate(
    const std::vector<double>& x,
    const std::vector<double>& y,
    double xi) const
{
    if (xi <= x.front())
        return y.front();
    if (xi >= x.back())
        return y.back();

    for (size_t i = 1; i < x.size(); ++i) {
        if (xi <= x[i]) {
            double t = (xi - x[i - 1]) / (x[i] - x[i - 1]);
            return y[i - 1] + t * (y[i] - y[i - 1]);
        }
    }

    return y.back();
}

double ThermalProfileModel::computeSoundSpeed(
    double temp_C,
    double salinity_ppt,
    double depth_m)
{
    // Mackenzie equation for sound speed in seawater
    double T = temp_C;
    double S = salinity_ppt;
    double D = depth_m;

    double c = 1448.96 + 4.591 * T - 5.304e-2 * T * T + 2.374e-4 * T * T * T +
               1.340 * (S - 35.0) + 1.630e-2 * D + 1.675e-7 * D * D -
               1.025e-2 * T * (S - 35.0) - 7.139e-13 * T * D * D * D;

    return c;
}

// ============================================================================
// Bathymetry (Sprint 9.2.3)
// ============================================================================

BathymetryModel::BathymetryModel() : reference_depth_(100.0), is_flat_(true) {}

void BathymetryModel::setFlatBottom(double depth_m)
{
    reference_depth_ = depth_m;
    is_flat_ = true;
}

void BathymetryModel::loadBathymetryData(const std::string& filename)
{
    (void)filename;
    // Placeholder for loading bathymetry grid data
    // In production: read GEBCO or similar bathymetry data
}

double BathymetryModel::getDepth(const Eigen::Vector3d& position) const
{
    (void)position;  // Could be used for spatial variation
    return reference_depth_;
}

Eigen::Vector3d BathymetryModel::getSeabedGradient(
    const Eigen::Vector3d& position) const
{
    (void)position;
    // Flat bottom has zero gradient
    return Eigen::Vector3d::Zero();
}

bool BathymetryModel::isDeepWater(const Eigen::Vector3d& position) const
{
    double depth = getDepth(position);
    // Deep water: depth > wavelength/2
    // Assume typical wavelength of 100m
    return depth > 50.0;
}

}  // namespace environment
}  // namespace lotusim
