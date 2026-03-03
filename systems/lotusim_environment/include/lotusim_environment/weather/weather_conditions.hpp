/**
 * @file weather_conditions.hpp
 * @brief Visibility and precipitation modeling
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <string>

namespace lotusim {
namespace environment {

enum class PrecipitationType
{
    NONE,
    RAIN,
    SNOW,
    SLEET,
    HAIL
};

enum class VisibilityCategory
{
    EXCELLENT,  // > 10 km
    GOOD,       // 5-10 km
    MODERATE,   // 2-5 km
    POOR,       // 1-2 km
    FOG,        // < 1 km
    DENSE_FOG   // < 200 m
};

struct WeatherConditions {
    double visibility_km{10.0};
    double precipitation_rate_mm_h{0.0};
    double cloud_cover_octas{0.0};  // 0-8
    double temperature_C{15.0};
    double humidity_percent{50.0};
    PrecipitationType precipitation_type{PrecipitationType::NONE};
};

class WeatherModel {
public:
    explicit WeatherModel(const WeatherConditions& cond = WeatherConditions());

    void update(double dt);
    void setConditions(const WeatherConditions& cond);

    double getVisibility() const
    {
        return conditions_.visibility_km;
    }
    double getPrecipitationRate() const
    {
        return conditions_.precipitation_rate_mm_h;
    }
    VisibilityCategory getVisibilityCategory() const;
    std::string getDescription() const;

    // Effects on sensors
    double getRadarAttenuation_dB() const;
    double getOpticalAttenuation() const;

private:
    WeatherConditions conditions_;
    double elapsed_time_{0.0};
};

}  // namespace environment
}  // namespace lotusim
