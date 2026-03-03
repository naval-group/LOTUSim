/**
 * @file acoustic_propagation.hpp
 * @brief Advanced underwater acoustic propagation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace lotusim {
namespace environment {

// Forward declaration
struct ThermalProfile;

// Sprint 9.3.2: Advanced acoustic propagation

/**
 * @brief Acoustic ray path
 */
struct AcousticRay {
    std::vector<Eigen::Vector3d> waypoints;  // Ray trajectory points
    std::vector<double> sound_speeds;        // Sound speed at each point (m/s)
    double path_loss_dB{0.0};                // Total path loss (dB)
    double travel_time_s{0.0};               // Travel time (seconds)
    int num_reflections{0};        // Number of bottom/surface reflections
    bool is_sofar_channel{false};  // True if confined to SOFAR channel
};

/**
 * @brief Eigenray (arrival path)
 */
struct Eigenray {
    AcousticRay ray;
    double launch_angle_deg{0.0};   // Initial angle from horizontal (degrees)
    double arrival_angle_deg{0.0};  // Final angle at receiver (degrees)
    double time_delay_s{0.0};       // Time delay relative to first arrival
};

/**
 * @brief Ray tracing results
 */
struct RayTracingResult {
    std::vector<Eigenray>
        eigenrays;  // All eigenrays connecting source and receiver
    double transmission_loss_dB{0.0};  // Total transmission loss (dB)
    bool is_shadow_zone{false};        // True if receiver is in shadow zone
};

/**
 * @brief Acoustic propagation model with ray tracing
 */
class AcousticPropagationModel {
public:
    /**
     * @brief Constructor
     * @param thermal_profile Thermal profile for sound speed
     */
    explicit AcousticPropagationModel(
        const ThermalProfile* thermal_profile = nullptr);

    /**
     * @brief Trace acoustic rays from source to receiver
     * @param source_pos Source position (NED, meters)
     * @param receiver_pos Receiver position (NED, meters)
     * @param frequency_hz Acoustic frequency (Hz)
     * @param water_depth_m Water depth (m)
     * @return Ray tracing results
     */
    RayTracingResult traceRays(
        const Eigen::Vector3d& source_pos,
        const Eigen::Vector3d& receiver_pos,
        double frequency_hz,
        double water_depth_m) const;

    /**
     * @brief Compute transmission loss using simple model
     * @param source_pos Source position (NED, meters)
     * @param receiver_pos Receiver position (NED, meters)
     * @param frequency_hz Frequency (Hz)
     * @return Transmission loss (dB)
     */
    double computeTransmissionLoss(
        const Eigen::Vector3d& source_pos,
        const Eigen::Vector3d& receiver_pos,
        double frequency_hz) const;

    /**
     * @brief Check if position is in SOFAR channel
     * @param depth_m Depth (m, positive downward)
     * @return True if in SOFAR channel
     */
    bool isInSofarChannel(double depth_m) const;

    /**
     * @brief Get SOFAR channel axis depth
     * @return Depth of minimum sound speed (m)
     */
    double getSofarAxisDepth() const;

    /**
     * @brief Set thermal profile
     * @param profile Thermal profile pointer
     */
    void setThermalProfile(const ThermalProfile* profile)
    {
        thermal_profile_ = profile;
    }

    /**
     * @brief Set sea surface roughness
     * @param significant_wave_height_m Significant wave height (m)
     */
    void setSeaSurfaceRoughness(double significant_wave_height_m)
    {
        sea_surface_roughness_m_ = significant_wave_height_m;
    }

    /**
     * @brief Set bottom type
     * @param type Bottom type ("sand", "mud", "rock", "gravel")
     */
    void setBottomType(const std::string& type);

    /**
     * @brief Get sound speed at depth
     * @param depth_m Depth (m, positive downward)
     * @return Sound speed (m/s)
     */
    double getSoundSpeed(double depth_m) const;

private:
    /**
     * @brief Trace single ray from source
     * @param source_pos Source position (NED, meters)
     * @param launch_angle_deg Launch angle from horizontal (degrees)
     * @param max_range_m Maximum range to trace (m)
     * @param water_depth_m Water depth (m)
     * @return Traced ray
     */
    AcousticRay traceSingleRay(
        const Eigen::Vector3d& source_pos,
        double launch_angle_deg,
        double max_range_m,
        double water_depth_m) const;

    /**
     * @brief Compute spreading loss (spherical + cylindrical)
     */
    double computeSpreadingLoss(double distance_m, double water_depth_m) const;

    /**
     * @brief Compute absorption loss
     */
    double computeAbsorptionLoss(double distance_m, double frequency_hz) const;

    /**
     * @brief Compute surface reflection loss
     */
    double computeSurfaceReflectionLoss() const;

    /**
     * @brief Compute bottom reflection loss
     */
    double computeBottomReflectionLoss(double grazing_angle_deg) const;

    /**
     * @brief Apply Snell's law for refraction
     * @param incident_angle_deg Incident angle from horizontal (degrees)
     * @param c1 Sound speed at current position (m/s)
     * @param c2 Sound speed at next position (m/s)
     * @return Refracted angle from horizontal (degrees)
     */
    double applySnellsLaw(double incident_angle_deg, double c1, double c2)
        const;

    // Thermal profile for sound speed
    const ThermalProfile* thermal_profile_;

    // Sea surface roughness (m)
    double sea_surface_roughness_m_{0.5};

    // Bottom properties
    std::string bottom_type_{"sand"};
    double bottom_loss_coeff_{0.5};  // Reflection loss coefficient

    // Ray tracing parameters
    static constexpr int NUM_RAYS = 181;        // Number of rays to trace
    static constexpr double RAY_STEP_M = 10.0;  // Step size for ray tracing (m)
};

/**
 * @brief SOFAR channel model
 */
class SofarChannelModel {
public:
    /**
     * @brief Constructor
     * @param thermal_profile Thermal profile
     */
    explicit SofarChannelModel(const ThermalProfile* thermal_profile);

    /**
     * @brief Compute SOFAR channel parameters
     */
    void computeChannelParameters();

    /**
     * @brief Get channel axis depth
     * @return Depth of minimum sound speed (m)
     */
    double getAxisDepth() const
    {
        return axis_depth_m_;
    }

    /**
     * @brief Get channel half-width
     * @return Half-width of SOFAR channel (m)
     */
    double getChannelHalfWidth() const
    {
        return channel_half_width_m_;
    }

    /**
     * @brief Check if depth is in channel
     * @param depth_m Depth (m)
     * @return True if in channel
     */
    bool isInChannel(double depth_m) const
    {
        return std::abs(depth_m - axis_depth_m_) <= channel_half_width_m_;
    }

    /**
     * @brief Compute propagation loss in SOFAR channel
     * @param distance_m Distance (m)
     * @param frequency_hz Frequency (Hz)
     * @return Transmission loss (dB)
     */
    double computeSofarLoss(double distance_m, double frequency_hz) const;

private:
    const ThermalProfile* thermal_profile_;
    double axis_depth_m_{1000.0};         // Channel axis depth (m)
    double channel_half_width_m_{200.0};  // Channel half-width (m)
    double min_sound_speed_mps_{1490.0};  // Minimum sound speed (m/s)
};

/**
 * @brief Absorption coefficient calculator (Francois-Garrison)
 */
class AbsorptionModel {
public:
    /**
     * @brief Compute absorption coefficient
     * @param frequency_hz Frequency (Hz)
     * @param temperature_C Temperature (°C)
     * @param salinity_ppt Salinity (ppt)
     * @param depth_m Depth (m)
     * @param pH pH value
     * @return Absorption coefficient (dB/km)
     */
    static double computeAbsorption(
        double frequency_hz,
        double temperature_C,
        double salinity_ppt,
        double depth_m,
        double pH = 8.0);

private:
    /**
     * @brief Boric acid contribution
     */
    static double boricAcidContribution(
        double frequency_hz,
        double temperature_C,
        double salinity_ppt,
        double depth_m,
        double pH);

    /**
     * @brief Magnesium sulfate contribution
     */
    static double magnesiumSulfateContribution(
        double frequency_hz,
        double temperature_C,
        double salinity_ppt,
        double depth_m);

    /**
     * @brief Pure water contribution
     */
    static double pureWaterContribution(
        double frequency_hz,
        double temperature_C,
        double salinity_ppt,
        double depth_m);
};

}  // namespace environment
}  // namespace lotusim
