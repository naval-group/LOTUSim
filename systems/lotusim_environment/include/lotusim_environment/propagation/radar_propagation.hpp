/**
 * @file radar_propagation.hpp
 * @brief Advanced radar propagation models
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace lotusim {
namespace environment {

// Sprint 9.3.1: Advanced radar propagation

/**
 * @brief Atmospheric refractivity profile
 */
struct RefractivityProfile {
    std::vector<double> altitude_m;  // Altitude samples (m)
    std::vector<double> N_units;     // Refractivity (N-units)
};

/**
 * @brief Atmospheric duct description
 */
struct AtmosphericDuct {
    double base_altitude_m{0.0};   // Duct base altitude (m)
    double top_altitude_m{100.0};  // Duct top altitude (m)
    double strength{1.0};          // Duct strength (0-1)
    bool is_surface_duct{true};    // True if surface duct
};

/**
 * @brief Radar ray path segment
 */
struct RaySegment {
    Eigen::Vector3d start;  // Start position (m)
    Eigen::Vector3d end;    // End position (m)
    double path_loss_dB{0.0};
};

/**
 * @brief Complete propagation path
 */
struct PropagationPath {
    std::vector<RaySegment> segments;
    double total_path_loss_dB{0.0};
    double total_distance_m{0.0};
    bool is_ducted{false};
    int num_reflections{0};
};

/**
 * @brief Radar propagation model with multipath and ducting
 */
class RadarPropagationModel {
public:
    RadarPropagationModel();

    /**
     * @brief Compute all propagation paths from transmitter to receiver
     * @param tx_pos Transmitter position (NED, meters)
     * @param rx_pos Receiver position (NED, meters)
     * @param frequency_hz Radar frequency (Hz)
     * @return Vector of propagation paths
     */
    std::vector<PropagationPath> computePaths(
        const Eigen::Vector3d& tx_pos,
        const Eigen::Vector3d& rx_pos,
        double frequency_hz) const;

    /**
     * @brief Get radar horizon distance
     * @param tx_altitude_m Transmitter altitude (m)
     * @param rx_altitude_m Receiver altitude (m)
     * @return Horizon distance (m)
     */
    double getRadarHorizon(double tx_altitude_m, double rx_altitude_m) const;

    /**
     * @brief Check if path is in duct
     * @param altitude_m Path altitude (m)
     * @return True if in duct
     */
    bool isInDuct(double altitude_m) const;

    /**
     * @brief Set atmospheric conditions
     * @param temperature_C Temperature (°C)
     * @param humidity_percent Relative humidity (%)
     * @param pressure_hPa Atmospheric pressure (hPa)
     */
    void setAtmosphericConditions(
        double temperature_C,
        double humidity_percent,
        double pressure_hPa);

    /**
     * @brief Set atmospheric duct
     * @param duct Duct parameters
     */
    void setDuct(const AtmosphericDuct& duct)
    {
        duct_ = duct;
        has_duct_ = true;
    }

    /**
     * @brief Clear atmospheric duct
     */
    void clearDuct()
    {
        has_duct_ = false;
    }

    /**
     * @brief Load refractivity profile
     * @param profile Refractivity profile
     */
    void setRefractivityProfile(const RefractivityProfile& profile)
    {
        refractivity_profile_ = profile;
    }

    /**
     * @brief Create standard atmosphere refractivity profile
     * @return Standard profile
     */
    static RefractivityProfile createStandardAtmosphere();

    /**
     * @brief Create evaporation duct profile
     * @param duct_height_m Duct height (m)
     * @return Evaporation duct profile
     */
    static RefractivityProfile createEvaporationDuct(double duct_height_m);

    // Constants (public for MultipathModel)
    static constexpr double EARTH_RADIUS_M = 6371000.0;
    static constexpr double STANDARD_K_FACTOR = 4.0 / 3.0;

private:
    /**
     * @brief Compute direct line-of-sight path
     */
    PropagationPath computeDirectPath(
        const Eigen::Vector3d& tx_pos,
        const Eigen::Vector3d& rx_pos,
        double frequency_hz) const;

    /**
     * @brief Compute surface reflection path
     */
    PropagationPath computeReflectionPath(
        const Eigen::Vector3d& tx_pos,
        const Eigen::Vector3d& rx_pos,
        double frequency_hz) const;

    /**
     * @brief Compute ducted propagation path
     */
    PropagationPath computeDuctedPath(
        const Eigen::Vector3d& tx_pos,
        const Eigen::Vector3d& rx_pos,
        double frequency_hz) const;

    /**
     * @brief Compute path loss for free space
     */
    double computeFreeSpaceLoss(double distance_m, double frequency_hz) const;

    /**
     * @brief Compute refractivity at altitude
     */
    double getRefractivity(double altitude_m) const;

    /**
     * @brief Compute refractive index gradient
     */
    double getRefractiveGradient(double altitude_m) const;

    /**
     * @brief Compute Earth's effective radius
     */
    double getEffectiveEarthRadius() const;

    /**
     * @brief Compute Fresnel zone clearance
     */
    double computeFresnelClearance(
        const Eigen::Vector3d& tx_pos,
        const Eigen::Vector3d& rx_pos,
        double frequency_hz) const;

    // Configuration
    double temperature_C_{15.0};
    double humidity_percent_{50.0};
    double pressure_hPa_{1013.25};

    // Atmospheric duct
    AtmosphericDuct duct_;
    bool has_duct_{false};

    // Refractivity profile
    RefractivityProfile refractivity_profile_;
};

/**
 * @brief Multipath interference calculator
 */
class MultipathModel {
public:
    /**
     * @brief Compute multipath fading
     * @param paths Vector of propagation paths
     * @param frequency_hz Radar frequency (Hz)
     * @return Additional path loss due to multipath (dB)
     */
    static double computeMultipathFading(
        const std::vector<PropagationPath>& paths,
        double frequency_hz);

    /**
     * @brief Compute Rayleigh fading statistics
     * @param num_paths Number of multipath components
     * @return Fading margin (dB) for 99% availability
     */
    static double getRayleighFadingMargin(int num_paths);

    /**
     * @brief Divergence factor for surface reflection
     * @param grazing_angle_rad Grazing angle (radians)
     * @param distance_m Distance to reflection point (m)
     * @param frequency_hz Frequency (Hz)
     * @return Divergence factor (< 1.0)
     */
    static double computeDivergenceFactor(
        double grazing_angle_rad,
        double distance_m,
        double frequency_hz);
};

}  // namespace environment
}  // namespace lotusim
