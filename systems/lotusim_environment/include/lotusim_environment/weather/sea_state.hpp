/**
 * @file sea_state.hpp
 * @brief Sea state modeling (Douglas scale)
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <string>

namespace lotusim {
namespace environment {

/**
 * @brief Douglas sea state scale (0-9)
 */
enum class DouglasSeaState
{
    CALM_GLASSY = 0,   // 0 m
    CALM_RIPPLED = 1,  // 0-0.1 m
    SMOOTH = 2,        // 0.1-0.5 m
    SLIGHT = 3,        // 0.5-1.25 m
    MODERATE = 4,      // 1.25-2.5 m
    ROUGH = 5,         // 2.5-4 m
    VERY_ROUGH = 6,    // 4-6 m
    HIGH = 7,          // 6-9 m
    VERY_HIGH = 8,     // 9-14 m
    PHENOMENAL = 9     // > 14 m
};

/**
 * @brief Sea state parameters
 */
struct SeaStateParameters {
    double significant_wave_height_m{2.0};
    double wind_speed_knots{15.0};
    double visibility_km{10.0};
    DouglasSeaState douglas{DouglasSeaState::MODERATE};
};

/**
 * @brief Sea state model
 */
class SeaState {
public:
    explicit SeaState(const SeaStateParameters& params = SeaStateParameters());

    void update(double dt);
    void setParameters(const SeaStateParameters& params);
    void setFromWaveHeight(double Hs_m);
    void setFromWindSpeed(double wind_speed_knots);

    DouglasSeaState getDouglasScale() const;
    double getSignificantWaveHeight() const
    {
        return params_.significant_wave_height_m;
    }
    double getWindSpeed() const
    {
        return params_.wind_speed_knots;
    }
    double getVisibility() const
    {
        return params_.visibility_km;
    }

    std::string getDescription() const;

    static DouglasSeaState waveHeightToDouglas(double Hs_m);
    static double douglasToWaveHeight(DouglasSeaState douglas);

private:
    SeaStateParameters params_;
    double elapsed_time_{0.0};
};

}  // namespace environment
}  // namespace lotusim
