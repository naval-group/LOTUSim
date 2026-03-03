/**
 * @file weather_conditions.cpp
 * @brief Visibility and precipitation implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/weather/weather_conditions.hpp"

#include <cmath>

namespace lotusim {
namespace environment {

WeatherModel::WeatherModel(const WeatherConditions& cond)
    : conditions_(cond), elapsed_time_(0.0)
{
}

void WeatherModel::update(double dt)
{
    elapsed_time_ += dt;
}

void WeatherModel::setConditions(const WeatherConditions& cond)
{
    conditions_ = cond;
}

VisibilityCategory WeatherModel::getVisibilityCategory() const
{
    if (conditions_.visibility_km < 0.2)
        return VisibilityCategory::DENSE_FOG;
    if (conditions_.visibility_km < 1.0)
        return VisibilityCategory::FOG;
    if (conditions_.visibility_km < 2.0)
        return VisibilityCategory::POOR;
    if (conditions_.visibility_km < 5.0)
        return VisibilityCategory::MODERATE;
    if (conditions_.visibility_km < 10.0)
        return VisibilityCategory::GOOD;
    return VisibilityCategory::EXCELLENT;
}

std::string WeatherModel::getDescription() const
{
    std::string desc;

    // Visibility
    switch (getVisibilityCategory()) {
        case VisibilityCategory::EXCELLENT:
            desc += "Excellent visibility";
            break;
        case VisibilityCategory::GOOD:
            desc += "Good visibility";
            break;
        case VisibilityCategory::MODERATE:
            desc += "Moderate visibility";
            break;
        case VisibilityCategory::POOR:
            desc += "Poor visibility";
            break;
        case VisibilityCategory::FOG:
            desc += "Fog";
            break;
        case VisibilityCategory::DENSE_FOG:
            desc += "Dense fog";
            break;
    }

    // Precipitation
    if (conditions_.precipitation_rate_mm_h > 0.1) {
        desc += ", ";
        switch (conditions_.precipitation_type) {
            case PrecipitationType::RAIN:
                if (conditions_.precipitation_rate_mm_h < 2.5)
                    desc += "light rain";
                else if (conditions_.precipitation_rate_mm_h < 10.0)
                    desc += "moderate rain";
                else
                    desc += "heavy rain";
                break;
            case PrecipitationType::SNOW:
                desc += "snow";
                break;
            case PrecipitationType::SLEET:
                desc += "sleet";
                break;
            case PrecipitationType::HAIL:
                desc += "hail";
                break;
            default:
                break;
        }
    }

    return desc;
}

double WeatherModel::getRadarAttenuation_dB() const
{
    // ITU-R P.838 approximation for rain attenuation at X-band (10 GHz)
    // γ = k * R^α  (dB/km)
    // where k=0.03, α=1.0 for rain

    if (conditions_.precipitation_type != PrecipitationType::RAIN) {
        return 0.0;
    }

    double R = conditions_.precipitation_rate_mm_h;
    double gamma_dB_km = 0.03 * std::pow(R, 1.0);  // Simplified model

    // Assume 10 km path for attenuation
    return gamma_dB_km * 10.0;
}

double WeatherModel::getOpticalAttenuation() const
{
    // Koschmieder equation: visibility = 3.912 / β
    // where β is extinction coefficient (1/m)

    double beta = 3.912 / (conditions_.visibility_km * 1000.0);

    // Transmission over 1 km: T = exp(-β * d)
    double transmission_1km = std::exp(-beta * 1000.0);

    // Return attenuation (1.0 = full attenuation, 0.0 = no attenuation)
    return 1.0 - transmission_1km;
}

}  // namespace environment
}  // namespace lotusim
