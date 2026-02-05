/**
 * @file wind_model.cpp
 * @brief Wind modeling implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/weather/wind_model.hpp"

#include <cmath>
#include <random>

namespace lotusim {
namespace environment {

WindModel::WindModel(const WindParameters& params)
    : params_(params)
    , elapsed_time_(0.0)
    , current_velocity_(Eigen::Vector3d::Zero())
{
    // Initialize velocity from parameters
    double speed_mps = knotsToMps(params_.speed_knots);
    double dir_rad = params_.direction_deg * M_PI / 180.0;

    // Wind direction: 0° = North, 90° = East
    // Convert to NED frame: North = +X, East = +Y
    current_velocity_.x() = speed_mps * std::cos(dir_rad);
    current_velocity_.y() = speed_mps * std::sin(dir_rad);
    current_velocity_.z() = 0.0;  // No vertical component for mean wind
}

void WindModel::update(double dt)
{
    elapsed_time_ += dt;

    // Base wind speed
    double speed_mps = knotsToMps(params_.speed_knots);
    double dir_rad = params_.direction_deg * M_PI / 180.0;

    // Apply gust multiplier (periodic)
    double gust_mult = computeGustMultiplier(elapsed_time_);
    double effective_speed = speed_mps * gust_mult;

    // Base wind vector
    Eigen::Vector3d base_wind;
    base_wind.x() = effective_speed * std::cos(dir_rad);
    base_wind.y() = effective_speed * std::sin(dir_rad);
    base_wind.z() = 0.0;

    // Add turbulence
    Eigen::Vector3d turbulence = computeTurbulence(elapsed_time_);

    current_velocity_ = base_wind + turbulence;
}

Eigen::Vector3d WindModel::getWindVelocity() const
{
    return current_velocity_;
}

double WindModel::getWindSpeed() const
{
    return current_velocity_.head<2>().norm();
}

double WindModel::getWindDirection() const
{
    double dir_rad = std::atan2(current_velocity_.y(), current_velocity_.x());
    double dir_deg = dir_rad * 180.0 / M_PI;
    return dir_deg;
}

BeaufortScale WindModel::getBeaufortScale() const
{
    double speed_knots = mpsToKnots(getWindSpeed());
    return speedToBeaufort(speed_knots);
}

void WindModel::setParameters(const WindParameters& params)
{
    params_ = params;
}

void WindModel::setFromBeaufort(BeaufortScale beaufort, double direction_deg)
{
    params_.beaufort = beaufort;
    params_.speed_knots = beaufortToSpeed(beaufort);
    params_.direction_deg = direction_deg;

    // Adjust gust factor based on Beaufort scale
    if (static_cast<int>(beaufort) >= static_cast<int>(BeaufortScale::GALE)) {
        params_.gust_factor = 1.5;
    } else if (
        static_cast<int>(beaufort) >=
        static_cast<int>(BeaufortScale::STRONG_BREEZE)) {
        params_.gust_factor = 1.3;
    } else {
        params_.gust_factor = 1.2;
    }

    // Reset time
    elapsed_time_ = 0.0;

    // Update current velocity
    double speed_mps = knotsToMps(params_.speed_knots);
    double dir_rad = params_.direction_deg * M_PI / 180.0;
    current_velocity_.x() = speed_mps * std::cos(dir_rad);
    current_velocity_.y() = speed_mps * std::sin(dir_rad);
    current_velocity_.z() = 0.0;
}

double WindModel::computeGustMultiplier(double time) const
{
    // Sinusoidal gust pattern
    double phase = 2.0 * M_PI * time / params_.gust_period_s;
    double gust_amplitude = params_.gust_factor - 1.0;
    return 1.0 + gust_amplitude * (0.5 + 0.5 * std::sin(phase));
}

Eigen::Vector3d WindModel::computeTurbulence(double time) const
{
    // Simple turbulence model using pseudo-random variations
    // In production, would use more sophisticated models (von Karman, Dryden)
    (void)time;  // Could be used for time-varying turbulence

    if (params_.turbulence_intensity < 1e-6) {
        return Eigen::Vector3d::Zero();
    }

    // Static random generator (for repeatability)
    static std::mt19937 gen(42);
    static std::normal_distribution<double> dist(0.0, 1.0);

    double base_speed = knotsToMps(params_.speed_knots);
    double turb_magnitude = base_speed * params_.turbulence_intensity;

    Eigen::Vector3d turbulence;
    turbulence.x() = turb_magnitude * dist(gen);
    turbulence.y() = turb_magnitude * dist(gen);
    turbulence.z() =
        turb_magnitude * 0.5 * dist(gen);  // Vertical turbulence smaller

    return turbulence;
}

double WindModel::beaufortToSpeed(BeaufortScale beaufort)
{
    // Beaufort scale to wind speed (knots, midpoint of range)
    switch (beaufort) {
        case BeaufortScale::CALM:
            return 0.0;
        case BeaufortScale::LIGHT_AIR:
            return 2.0;
        case BeaufortScale::LIGHT_BREEZE:
            return 5.0;
        case BeaufortScale::GENTLE_BREEZE:
            return 8.5;
        case BeaufortScale::MODERATE_BREEZE:
            return 13.5;
        case BeaufortScale::FRESH_BREEZE:
            return 19.0;
        case BeaufortScale::STRONG_BREEZE:
            return 24.5;
        case BeaufortScale::NEAR_GALE:
            return 30.5;
        case BeaufortScale::GALE:
            return 37.0;
        case BeaufortScale::STRONG_GALE:
            return 44.0;
        case BeaufortScale::STORM:
            return 51.5;
        case BeaufortScale::VIOLENT_STORM:
            return 59.5;
        case BeaufortScale::HURRICANE:
            return 70.0;
        default:
            return 0.0;
    }
}

BeaufortScale WindModel::speedToBeaufort(double speed_knots)
{
    if (speed_knots < 1.0)
        return BeaufortScale::CALM;
    if (speed_knots < 4.0)
        return BeaufortScale::LIGHT_AIR;
    if (speed_knots < 7.0)
        return BeaufortScale::LIGHT_BREEZE;
    if (speed_knots < 11.0)
        return BeaufortScale::GENTLE_BREEZE;
    if (speed_knots < 17.0)
        return BeaufortScale::MODERATE_BREEZE;
    if (speed_knots < 22.0)
        return BeaufortScale::FRESH_BREEZE;
    if (speed_knots < 28.0)
        return BeaufortScale::STRONG_BREEZE;
    if (speed_knots < 34.0)
        return BeaufortScale::NEAR_GALE;
    if (speed_knots < 41.0)
        return BeaufortScale::GALE;
    if (speed_knots < 48.0)
        return BeaufortScale::STRONG_GALE;
    if (speed_knots < 56.0)
        return BeaufortScale::STORM;
    if (speed_knots < 64.0)
        return BeaufortScale::VIOLENT_STORM;
    return BeaufortScale::HURRICANE;
}

}  // namespace environment
}  // namespace lotusim
