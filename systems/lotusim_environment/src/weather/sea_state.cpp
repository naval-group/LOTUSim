/**
 * @file sea_state.cpp
 * @brief Sea state modeling implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/weather/sea_state.hpp"

#include <cmath>

namespace lotusim {
namespace environment {

SeaState::SeaState(const SeaStateParameters& params)
    : params_(params), elapsed_time_(0.0)
{
}

void SeaState::update(double dt)
{
    elapsed_time_ += dt;
}

void SeaState::setParameters(const SeaStateParameters& params)
{
    params_ = params;
}

void SeaState::setFromWaveHeight(double Hs_m)
{
    params_.significant_wave_height_m = Hs_m;
    params_.douglas = waveHeightToDouglas(Hs_m);
}

void SeaState::setFromWindSpeed(double wind_speed_knots)
{
    params_.wind_speed_knots = wind_speed_knots;

    // Empirical relationship: Hs ≈ 0.21 * U10² / g (fully developed)
    double U10 = wind_speed_knots * 0.514444;  // knots to m/s
    double Hs = 0.21 * U10 * U10 / 9.81;

    params_.significant_wave_height_m = Hs;
    params_.douglas = waveHeightToDouglas(Hs);
}

DouglasSeaState SeaState::getDouglasScale() const
{
    return waveHeightToDouglas(params_.significant_wave_height_m);
}

std::string SeaState::getDescription() const
{
    switch (getDouglasScale()) {
        case DouglasSeaState::CALM_GLASSY:
            return "Calm (glassy)";
        case DouglasSeaState::CALM_RIPPLED:
            return "Calm (rippled)";
        case DouglasSeaState::SMOOTH:
            return "Smooth";
        case DouglasSeaState::SLIGHT:
            return "Slight";
        case DouglasSeaState::MODERATE:
            return "Moderate";
        case DouglasSeaState::ROUGH:
            return "Rough";
        case DouglasSeaState::VERY_ROUGH:
            return "Very rough";
        case DouglasSeaState::HIGH:
            return "High";
        case DouglasSeaState::VERY_HIGH:
            return "Very high";
        case DouglasSeaState::PHENOMENAL:
            return "Phenomenal";
        default:
            return "Unknown";
    }
}

DouglasSeaState SeaState::waveHeightToDouglas(double Hs_m)
{
    if (Hs_m < 0.05)
        return DouglasSeaState::CALM_GLASSY;
    if (Hs_m < 0.1)
        return DouglasSeaState::CALM_RIPPLED;
    if (Hs_m < 0.5)
        return DouglasSeaState::SMOOTH;
    if (Hs_m < 1.25)
        return DouglasSeaState::SLIGHT;
    if (Hs_m < 2.5)
        return DouglasSeaState::MODERATE;
    if (Hs_m < 4.0)
        return DouglasSeaState::ROUGH;
    if (Hs_m < 6.0)
        return DouglasSeaState::VERY_ROUGH;
    if (Hs_m < 9.0)
        return DouglasSeaState::HIGH;
    if (Hs_m < 14.0)
        return DouglasSeaState::VERY_HIGH;
    return DouglasSeaState::PHENOMENAL;
}

double SeaState::douglasToWaveHeight(DouglasSeaState douglas)
{
    switch (douglas) {
        case DouglasSeaState::CALM_GLASSY:
            return 0.0;
        case DouglasSeaState::CALM_RIPPLED:
            return 0.05;
        case DouglasSeaState::SMOOTH:
            return 0.3;
        case DouglasSeaState::SLIGHT:
            return 0.875;
        case DouglasSeaState::MODERATE:
            return 1.875;
        case DouglasSeaState::ROUGH:
            return 3.25;
        case DouglasSeaState::VERY_ROUGH:
            return 5.0;
        case DouglasSeaState::HIGH:
            return 7.5;
        case DouglasSeaState::VERY_HIGH:
            return 11.5;
        case DouglasSeaState::PHENOMENAL:
            return 16.0;
        default:
            return 0.0;
    }
}

}  // namespace environment
}  // namespace lotusim
