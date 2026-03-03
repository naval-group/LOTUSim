/**
 * @file ocean_model.hpp
 * @brief Ocean currents, thermal profiles, and bathymetry
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace lotusim {
namespace environment {

// Sprint 9.2.1: Ocean currents
struct CurrentParameters {
    Eigen::Vector3d surface_velocity{1.0, 0.0, 0.0};  // m/s (North, East, Down)
    double depth_decay_coefficient{0.02};             // 1/m
    double tidal_amplitude{0.5};                      // m/s
    double tidal_period_h{12.42};                     // hours (M2 tide)
};

class OceanCurrentModel {
public:
    explicit OceanCurrentModel(
        const CurrentParameters& params = CurrentParameters());

    void update(double dt);
    Eigen::Vector3d getCurrentVelocity(
        const Eigen::Vector3d& position,
        double depth) const;

    void setParameters(const CurrentParameters& params)
    {
        params_ = params;
    }

private:
    CurrentParameters params_;
    double elapsed_time_{0.0};
};

// Sprint 9.2.2: Thermal profiles
struct ThermalProfile {
    std::vector<double> depth_m;          // Depth samples
    std::vector<double> temperature_C;    // Temperature at each depth
    std::vector<double> sound_speed_mps;  // Speed of sound at each depth
};

class ThermalProfileModel {
public:
    ThermalProfileModel();

    void loadProfile(const std::string& region, const std::string& season);
    void setProfile(const ThermalProfile& profile);

    double getTemperature(double depth) const;
    double getSoundSpeed(double depth) const;
    double getThermoclineDepth() const;

    const ThermalProfile& getProfile() const
    {
        return profile_;
    }

    static ThermalProfile createDefaultProfile();
    static ThermalProfile createMediterraneanSummer();
    static ThermalProfile createAtlanticWinter();

    static double
    computeSoundSpeed(double temp_C, double salinity_ppt, double depth_m);

private:
    double interpolate(
        const std::vector<double>& x,
        const std::vector<double>& y,
        double xi) const;

    ThermalProfile profile_;
};

// Sprint 9.2.3: Bathymetry
class BathymetryModel {
public:
    BathymetryModel();

    void setFlatBottom(double depth_m);
    void loadBathymetryData(const std::string& filename);

    double getDepth(const Eigen::Vector3d& position) const;
    Eigen::Vector3d getSeabedGradient(const Eigen::Vector3d& position) const;

    bool isDeepWater(const Eigen::Vector3d& position) const;

private:
    double reference_depth_{100.0};  // Default depth
    bool is_flat_{true};
};

}  // namespace environment
}  // namespace lotusim
