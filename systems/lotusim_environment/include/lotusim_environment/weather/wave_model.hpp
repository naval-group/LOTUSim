/**
 * @file wave_model.hpp
 * @brief Ocean wave modeling for maritime simulation
 *
 * Implements Pierson-Moskowitz and JONSWAP wave spectra.
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace lotusim {
namespace environment {

/**
 * @brief Wave spectrum type
 */
enum class WaveSpectrumType
{
    PIERSON_MOSKOWITZ,  // Fully developed sea
    JONSWAP             // Fetch-limited sea
};

/**
 * @brief Wave parameters
 */
struct WaveParameters {
    double significant_wave_height_m{2.0};  // H_s (m)
    double peak_period_s{8.0};              // T_p (s)
    double wave_direction_deg{0.0};         // Direction (0=North, 90=East)
    double directional_spread_deg{30.0};    // Angular spread
    WaveSpectrumType spectrum{WaveSpectrumType::PIERSON_MOSKOWITZ};
    double gamma{3.3};  // JONSWAP peakedness parameter (1.0-7.0)
};

/**
 * @brief Wave component (for spectral decomposition)
 */
struct WaveComponent {
    double amplitude;    // Wave amplitude (m)
    double frequency;    // Angular frequency (rad/s)
    double wave_number;  // Wave number (rad/m)
    double direction;    // Direction (radians)
    double phase;        // Initial phase (radians)
};

/**
 * @brief Wave model with spectral decomposition
 */
class WaveModel {
public:
    /**
     * @brief Construct wave model from parameters
     */
    explicit WaveModel(const WaveParameters& params = WaveParameters());

    /**
     * @brief Update wave state
     * @param dt Time step in seconds
     */
    void update(double dt);

    /**
     * @brief Get wave elevation at position
     * @param position Position in world frame (NED)
     * @return Wave elevation in meters (positive = up)
     */
    double getElevation(const Eigen::Vector3d& position) const;

    /**
     * @brief Get wave velocity at position
     * @param position Position in world frame (NED)
     * @param depth Depth below surface (positive = down)
     * @return Wave orbital velocity [m/s]
     */
    Eigen::Vector3d getOrbitalVelocity(
        const Eigen::Vector3d& position,
        double depth) const;

    /**
     * @brief Get significant wave height
     * @return H_s in meters
     */
    double getSignificantWaveHeight() const
    {
        return params_.significant_wave_height_m;
    }

    /**
     * @brief Get peak period
     * @return T_p in seconds
     */
    double getPeakPeriod() const
    {
        return params_.peak_period_s;
    }

    /**
     * @brief Set wave parameters
     */
    void setParameters(const WaveParameters& params);

    /**
     * @brief Set waves from wind speed
     * @param wind_speed_knots Wind speed in knots
     * @param fetch_km Fetch distance in km (for JONSWAP)
     */
    void setFromWind(double wind_speed_knots, double fetch_km = 1000.0);

    /**
     * @brief Get wave components (for visualization)
     */
    const std::vector<WaveComponent>& getComponents() const
    {
        return components_;
    }

private:
    /**
     * @brief Generate wave components from spectrum
     */
    void generateComponents();

    /**
     * @brief Compute Pierson-Moskowitz spectrum
     * @param omega Angular frequency (rad/s)
     * @return Spectral density S(ω)
     */
    double spectrumPiersonMoskowitz(double omega) const;

    /**
     * @brief Compute JONSWAP spectrum
     * @param omega Angular frequency (rad/s)
     * @return Spectral density S(ω)
     */
    double spectrumJONSWAP(double omega) const;

    /**
     * @brief Get spectral density at frequency
     */
    double getSpectralDensity(double omega) const;

    /**
     * @brief Compute dispersion relation (deep water)
     * @param omega Angular frequency (rad/s)
     * @return Wave number k (rad/m)
     */
    static double dispersionRelation(double omega);

    WaveParameters params_;
    std::vector<WaveComponent> components_;
    double elapsed_time_{0.0};

    static constexpr double g_ = 9.81;          // Gravity (m/s²)
    static constexpr int num_components_ = 50;  // Number of wave components
};

}  // namespace environment
}  // namespace lotusim
