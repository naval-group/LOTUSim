/**
 * @file optical_propagation.hpp
 * @brief Optical propagation through atmosphere and water
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma once

#include <Eigen/Dense>

namespace lotusim {
namespace environment {

// Sprint 9.3.3: Optical propagation

/**
 * @brief Optical wavelength bands
 */
enum class OpticalBand
{
    VISIBLE,  // 400-700 nm
    NEAR_IR,  // 700-1400 nm
    MID_IR,   // 3-5 μm
    LONG_IR   // 8-14 μm (thermal)
};

/**
 * @brief Atmospheric optical properties
 */
struct AtmosphericOpticalProperties {
    double visibility_km{10.0};              // Visibility (km)
    double aerosol_optical_depth{0.1};       // AOD at 550 nm
    double water_vapor_cm{2.0};              // Precipitable water vapor (cm)
    double relative_humidity_percent{50.0};  // Relative humidity (%)
};

/**
 * @brief Underwater optical properties
 */
struct UnderwaterOpticalProperties {
    double attenuation_coeff_m{0.05};   // Beam attenuation coefficient (1/m)
    double scattering_coeff_m{0.03};    // Scattering coefficient (1/m)
    double absorption_coeff_m{0.02};    // Absorption coefficient (1/m)
    double turbidity_NTU{1.0};          // Turbidity (NTU)
    std::string water_type{"coastal"};  // "clear", "coastal", "harbor"
};

/**
 * @brief Atmospheric optical propagation model
 */
class AtmosphericOpticalModel {
public:
    AtmosphericOpticalModel();

    /**
     * @brief Compute atmospheric transmittance
     * @param distance_m Path distance (m)
     * @param wavelength_nm Wavelength (nm)
     * @return Transmittance (0-1)
     */
    double computeTransmittance(double distance_m, double wavelength_nm) const;

    /**
     * @brief Compute path radiance (scattering into line of sight)
     * @param distance_m Path distance (m)
     * @param wavelength_nm Wavelength (nm)
     * @param sun_zenith_deg Sun zenith angle (degrees)
     * @return Path radiance (W/m²/sr/nm)
     */
    double computePathRadiance(
        double distance_m,
        double wavelength_nm,
        double sun_zenith_deg) const;

    /**
     * @brief Apply refraction at air-water interface
     * @param incident_angle_deg Incident angle from normal (degrees)
     * @param from_air True if ray goes from air to water
     * @return Refracted angle from normal (degrees)
     */
    double applyRefraction(double incident_angle_deg, bool from_air) const;

    /**
     * @brief Compute Fresnel reflection coefficient
     * @param incident_angle_deg Incident angle from normal (degrees)
     * @param from_air True if ray goes from air to water
     * @return Reflection coefficient (0-1)
     */
    double computeFresnelReflection(double incident_angle_deg, bool from_air)
        const;

    /**
     * @brief Set atmospheric properties
     * @param props Atmospheric optical properties
     */
    void setAtmosphericProperties(const AtmosphericOpticalProperties& props)
    {
        atm_props_ = props;
    }

    /**
     * @brief Get extinction coefficient (Beer-Lambert law)
     * @param wavelength_nm Wavelength (nm)
     * @return Extinction coefficient (1/m)
     */
    double getExtinctionCoefficient(double wavelength_nm) const;

private:
    /**
     * @brief Rayleigh scattering coefficient
     */
    double computeRayleighScattering(double wavelength_nm) const;

    /**
     * @brief Mie scattering coefficient (aerosols)
     */
    double computeMieScattering(double wavelength_nm) const;

    /**
     * @brief Molecular absorption coefficient
     */
    double computeMolecularAbsorption(double wavelength_nm) const;

    AtmosphericOpticalProperties atm_props_;

    // Refractive indices
    static constexpr double n_air_ = 1.000293;
    static constexpr double n_water_ = 1.333;
};

/**
 * @brief Underwater optical propagation model
 */
class UnderwaterOpticalModel {
public:
    UnderwaterOpticalModel();

    /**
     * @brief Compute underwater transmittance (Beer-Lambert law)
     * @param distance_m Path distance (m)
     * @param wavelength_nm Wavelength (nm)
     * @return Transmittance (0-1)
     */
    double computeTransmittance(double distance_m, double wavelength_nm) const;

    /**
     * @brief Compute veiling light (backscatter)
     * @param distance_m Path distance (m)
     * @param wavelength_nm Wavelength (nm)
     * @return Veiling light radiance (W/m²/sr/nm)
     */
    double computeVeilingLight(double distance_m, double wavelength_nm) const;

    /**
     * @brief Compute contrast transmittance
     * @param distance_m Path distance (m)
     * @param wavelength_nm Wavelength (nm)
     * @return Contrast transmittance (0-1)
     */
    double computeContrastTransmittance(double distance_m, double wavelength_nm)
        const;

    /**
     * @brief Get maximum visible range
     * @param contrast_threshold Contrast threshold (typical: 0.02)
     * @param wavelength_nm Wavelength (nm)
     * @return Maximum range (m)
     */
    double getMaximumRange(double contrast_threshold, double wavelength_nm)
        const;

    /**
     * @brief Set underwater optical properties
     * @param props Underwater optical properties
     */
    void setUnderwaterProperties(const UnderwaterOpticalProperties& props)
    {
        water_props_ = props;
        updateCoefficients();
    }

    /**
     * @brief Create properties for water type
     * @param water_type "clear", "coastal", or "harbor"
     * @return Optical properties
     */
    static UnderwaterOpticalProperties createWaterType(
        const std::string& water_type);

private:
    /**
     * @brief Update coefficients based on water type
     */
    void updateCoefficients();

    /**
     * @brief Wavelength-dependent attenuation
     */
    double getAttenuationCoeff(double wavelength_nm) const;

    /**
     * @brief Wavelength-dependent scattering
     */
    double getScatteringCoeff(double wavelength_nm) const;

    UnderwaterOpticalProperties water_props_;
};

/**
 * @brief Combined optical propagation model (air + water)
 */
class OpticalPropagationModel {
public:
    OpticalPropagationModel();

    /**
     * @brief Compute total transmittance through air and water
     * @param air_distance_m Distance through air (m)
     * @param water_distance_m Distance through water (m)
     * @param wavelength_nm Wavelength (nm)
     * @param air_water_angle_deg Angle at air-water interface from normal
     * (degrees)
     * @return Total transmittance (0-1)
     */
    double computeTransmittance(
        double air_distance_m,
        double water_distance_m,
        double wavelength_nm,
        double air_water_angle_deg) const;

    /**
     * @brief Compute transmittance for specific optical band
     * @param air_distance_m Distance through air (m)
     * @param water_distance_m Distance through water (m)
     * @param band Optical wavelength band
     * @param air_water_angle_deg Angle at interface (degrees)
     * @return Band-averaged transmittance (0-1)
     */
    double computeBandTransmittance(
        double air_distance_m,
        double water_distance_m,
        OpticalBand band,
        double air_water_angle_deg) const;

    /**
     * @brief Set atmospheric properties
     */
    void setAtmosphericProperties(const AtmosphericOpticalProperties& props)
    {
        atm_model_.setAtmosphericProperties(props);
    }

    /**
     * @brief Set underwater properties
     */
    void setUnderwaterProperties(const UnderwaterOpticalProperties& props)
    {
        water_model_.setUnderwaterProperties(props);
    }

    /**
     * @brief Get wavelength range for band
     * @param band Optical band
     * @return Pair of (min_wavelength_nm, max_wavelength_nm)
     */
    static std::pair<double, double> getBandWavelengths(OpticalBand band);

private:
    AtmosphericOpticalModel atm_model_;
    UnderwaterOpticalModel water_model_;
};

}  // namespace environment
}  // namespace lotusim
