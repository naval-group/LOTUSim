/**
 * @file wave_model.cpp
 * @brief Ocean wave modeling implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/weather/wave_model.hpp"

#include <cmath>
#include <random>

namespace lotusim {
namespace environment {

WaveModel::WaveModel(const WaveParameters& params)
    : params_(params), elapsed_time_(0.0)
{
    generateComponents();
}

void WaveModel::update(double dt)
{
    elapsed_time_ += dt;
}

double WaveModel::getElevation(const Eigen::Vector3d& position) const
{
    double elevation = 0.0;

    for (const auto& comp : components_) {
        // Wave phase: k·x - ωt + φ
        double kx =
            comp.wave_number * (position.x() * std::cos(comp.direction) +
                                position.y() * std::sin(comp.direction));
        double phase = kx - comp.frequency * elapsed_time_ + comp.phase;

        elevation += comp.amplitude * std::cos(phase);
    }

    return elevation;
}

Eigen::Vector3d WaveModel::getOrbitalVelocity(
    const Eigen::Vector3d& position,
    double depth) const
{
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    for (const auto& comp : components_) {
        // Wave phase
        double kx =
            comp.wave_number * (position.x() * std::cos(comp.direction) +
                                position.y() * std::sin(comp.direction));
        double phase = kx - comp.frequency * elapsed_time_ + comp.phase;

        // Exponential decay with depth (deep water approximation)
        double decay = std::exp(-comp.wave_number * depth);

        // Horizontal orbital velocity
        double u_mag =
            comp.amplitude * comp.frequency * decay * std::cos(phase);

        velocity.x() += u_mag * std::cos(comp.direction);
        velocity.y() += u_mag * std::sin(comp.direction);

        // Vertical orbital velocity
        velocity.z() +=
            comp.amplitude * comp.frequency * decay * std::sin(phase);
    }

    return velocity;
}

void WaveModel::setParameters(const WaveParameters& params)
{
    params_ = params;
    generateComponents();
}

void WaveModel::setFromWind(double wind_speed_knots, double fetch_km)
{
    // Convert wind speed to m/s
    double U10 = wind_speed_knots * 0.514444;

    if (fetch_km < 500.0) {
        // Fetch-limited: use JONSWAP
        params_.spectrum = WaveSpectrumType::JONSWAP;

        // Empirical formulas for JONSWAP from fetch
        double F = fetch_km * 1000.0;  // Fetch in meters
        double g = 9.81;

        // Significant wave height (Hasselmann et al. 1973)
        double Hs =
            0.0016 * std::sqrt(g * F) * std::pow(U10 / std::sqrt(g * F), 0.5);

        // Peak period
        double Tp =
            0.286 * std::sqrt(g * F) * std::pow(U10 / std::sqrt(g * F), -0.33);

        params_.significant_wave_height_m = Hs;
        params_.peak_period_s = Tp;
        params_.gamma = 3.3;  // Standard JONSWAP peakedness
    } else {
        // Fully developed: use Pierson-Moskowitz
        params_.spectrum = WaveSpectrumType::PIERSON_MOSKOWITZ;

        // Pierson-Moskowitz formulas
        double g = 9.81;

        // Significant wave height
        params_.significant_wave_height_m = 0.21 * U10 * U10 / g;

        // Peak period
        params_.peak_period_s = 0.77 * U10 / g;
    }

    generateComponents();
}

void WaveModel::generateComponents()
{
    components_.clear();

    // Peak frequency
    double omega_p = 2.0 * M_PI / params_.peak_period_s;

    // Frequency range: 0.5*ω_p to 3.0*ω_p
    double omega_min = 0.5 * omega_p;
    double omega_max = 3.0 * omega_p;
    double d_omega = (omega_max - omega_min) / num_components_;

    std::mt19937 gen(42);  // Fixed seed for repeatability
    std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * M_PI);
    std::normal_distribution<double> dir_dist(
        0.0,
        params_.directional_spread_deg * M_PI / 180.0);

    for (int i = 0; i < num_components_; ++i) {
        double omega = omega_min + (i + 0.5) * d_omega;

        // Spectral density
        double S_omega = getSpectralDensity(omega);

        // Wave amplitude from spectral density
        // A_i = √(2 * S(ω_i) * Δω)
        double amplitude = std::sqrt(2.0 * S_omega * d_omega);

        // Wave number from dispersion relation
        double k = dispersionRelation(omega);

        // Direction (mean + spread)
        double mean_dir = params_.wave_direction_deg * M_PI / 180.0;
        double direction = mean_dir + dir_dist(gen);

        // Random phase
        double phase = phase_dist(gen);

        WaveComponent comp;
        comp.amplitude = amplitude;
        comp.frequency = omega;
        comp.wave_number = k;
        comp.direction = direction;
        comp.phase = phase;

        components_.push_back(comp);
    }
}

double WaveModel::spectrumPiersonMoskowitz(double omega) const
{
    double omega_p = 2.0 * M_PI / params_.peak_period_s;

    // Pierson-Moskowitz spectrum
    // S(ω) = (α g² / ω⁵) exp(-β (ω_p / ω)⁴)
    // where α = 0.0081, β = 1.25

    double alpha = 0.0081;
    double beta = 1.25;

    double S = (alpha * g_ * g_) / std::pow(omega, 5.0) *
               std::exp(-beta * std::pow(omega_p / omega, 4.0));

    return S;
}

double WaveModel::spectrumJONSWAP(double omega) const
{
    double omega_p = 2.0 * M_PI / params_.peak_period_s;

    // JONSWAP spectrum
    // S(ω) = α_PM g² / ω⁵ exp(-1.25 (ω_p / ω)⁴) γ^exp(-(ω-ω_p)²/(2σ²ω_p²))

    // Pierson-Moskowitz part
    double S_PM = spectrumPiersonMoskowitz(omega);

    // Peak enhancement factor
    double sigma = (omega <= omega_p) ? 0.07 : 0.09;
    double exponent = -std::pow(omega - omega_p, 2.0) /
                      (2.0 * sigma * sigma * omega_p * omega_p);
    double peak_enhancement = std::pow(params_.gamma, std::exp(exponent));

    return S_PM * peak_enhancement;
}

double WaveModel::getSpectralDensity(double omega) const
{
    switch (params_.spectrum) {
        case WaveSpectrumType::PIERSON_MOSKOWITZ:
            return spectrumPiersonMoskowitz(omega);
        case WaveSpectrumType::JONSWAP:
            return spectrumJONSWAP(omega);
        default:
            return 0.0;
    }
}

double WaveModel::dispersionRelation(double omega)
{
    // Deep water dispersion relation: ω² = gk
    // k = ω² / g
    return omega * omega / g_;
}

}  // namespace environment
}  // namespace lotusim
