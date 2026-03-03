/**
 * @file wind_model.hpp
 * @brief Wind modeling for maritime simulation
 *
 * Implements Beaufort scale, gusts, and wind field dynamics.
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>
#include <string>

namespace lotusim {
namespace environment {

/**
 * @brief Beaufort wind force scale (0-12)
 */
enum class BeaufortScale
{
    CALM = 0,             // < 1 kt
    LIGHT_AIR = 1,        // 1-3 kt
    LIGHT_BREEZE = 2,     // 4-6 kt
    GENTLE_BREEZE = 3,    // 7-10 kt
    MODERATE_BREEZE = 4,  // 11-16 kt
    FRESH_BREEZE = 5,     // 17-21 kt
    STRONG_BREEZE = 6,    // 22-27 kt
    NEAR_GALE = 7,        // 28-33 kt
    GALE = 8,             // 34-40 kt
    STRONG_GALE = 9,      // 41-47 kt
    STORM = 10,           // 48-55 kt
    VIOLENT_STORM = 11,   // 56-63 kt
    HURRICANE = 12        // > 63 kt
};

/**
 * @brief Wind parameters
 */
struct WindParameters {
    double speed_knots{10.0};          // Wind speed in knots
    double direction_deg{0.0};         // Wind direction (0=North, 90=East)
    double gust_factor{1.3};           // Gust multiplier (1.0-2.0)
    double gust_period_s{30.0};        // Gust period in seconds
    double turbulence_intensity{0.1};  // Turbulence intensity (0-1)
    BeaufortScale beaufort{BeaufortScale::GENTLE_BREEZE};
};

/**
 * @brief Wind model with temporal variation and gusts
 */
class WindModel {
public:
    /**
     * @brief Construct wind model from parameters
     */
    explicit WindModel(const WindParameters& params = WindParameters());

    /**
     * @brief Update wind state
     * @param dt Time step in seconds
     */
    void update(double dt);

    /**
     * @brief Get current wind velocity (world frame)
     * @return Wind velocity vector [m/s] (North, East, Up)
     */
    Eigen::Vector3d getWindVelocity() const;

    /**
     * @brief Get current wind speed
     * @return Wind speed in m/s
     */
    double getWindSpeed() const;

    /**
     * @brief Get current wind direction
     * @return Direction in degrees (0=North, 90=East)
     */
    double getWindDirection() const;

    /**
     * @brief Get Beaufort scale
     * @return Beaufort force
     */
    BeaufortScale getBeaufortScale() const;

    /**
     * @brief Set wind parameters
     */
    void setParameters(const WindParameters& params);

    /**
     * @brief Set wind from Beaufort scale
     */
    void setFromBeaufort(BeaufortScale beaufort, double direction_deg = 0.0);

private:
    /**
     * @brief Compute gust multiplier
     */
    double computeGustMultiplier(double time) const;

    /**
     * @brief Compute turbulence component
     */
    Eigen::Vector3d computeTurbulence(double time) const;

    /**
     * @brief Convert Beaufort to wind speed
     */
    static double beaufortToSpeed(BeaufortScale beaufort);

    /**
     * @brief Convert wind speed to Beaufort
     */
    static BeaufortScale speedToBeaufort(double speed_knots);

    WindParameters params_;
    double elapsed_time_{0.0};
    Eigen::Vector3d current_velocity_;
};

/**
 * @brief Convert knots to m/s
 */
inline double knotsToMps(double knots)
{
    return knots * 0.514444;
}

/**
 * @brief Convert m/s to knots
 */
inline double mpsToKnots(double mps)
{
    return mps / 0.514444;
}

}  // namespace environment
}  // namespace lotusim
