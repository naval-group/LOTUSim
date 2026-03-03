/**
 * @file optical_propagation.cpp
 * @brief Optical propagation implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/propagation/optical_propagation.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim {
namespace environment {

// ============================================================================
// AtmosphericOpticalModel Implementation
// ============================================================================

AtmosphericOpticalModel::AtmosphericOpticalModel() {}

double AtmosphericOpticalModel::computeTransmittance(
    double distance_m,
    double wavelength_nm) const
{
    // Beer-Lambert law: T = exp(-β·d)
    // beta is in 1/km, distance_km is in km
    double beta = getExtinctionCoefficient(wavelength_nm);
    double distance_km = distance_m / 1000.0;

    return std::exp(-beta * distance_km);
}

double AtmosphericOpticalModel::computePathRadiance(
    double distance_m,
    double wavelength_nm,
    double sun_zenith_deg) const
{
    // Simplified path radiance model
    // Depends on scattering and sun angle

    double beta_sca = computeRayleighScattering(wavelength_nm) +
                      computeMieScattering(wavelength_nm);

    double distance_km = distance_m / 1000.0;

    // Sun illumination factor
    double cos_zenith = std::cos(sun_zenith_deg * M_PI / 180.0);
    double E0 = 1000.0;  // Solar irradiance (W/m²)

    // Path radiance (simplified)
    double Lp =
        E0 * beta_sca * distance_km * std::max(0.0, cos_zenith) / (4.0 * M_PI);

    return Lp;
}

double AtmosphericOpticalModel::applyRefraction(
    double incident_angle_deg,
    bool from_air) const
{
    // Snell's law: n1·sin(θ1) = n2·sin(θ2)

    double n1 = from_air ? n_air_ : n_water_;
    double n2 = from_air ? n_water_ : n_air_;

    double theta1_rad = incident_angle_deg * M_PI / 180.0;
    double sin_theta1 = std::sin(theta1_rad);

    double sin_theta2 = (n1 / n2) * sin_theta1;

    // Check for total internal reflection
    if (std::abs(sin_theta2) > 1.0) {
        return 90.0;  // Total internal reflection
    }

    double theta2_rad = std::asin(sin_theta2);
    return theta2_rad * 180.0 / M_PI;
}

double AtmosphericOpticalModel::computeFresnelReflection(
    double incident_angle_deg,
    bool from_air) const
{
    // Fresnel equations for unpolarized light

    double n1 = from_air ? n_air_ : n_water_;
    double n2 = from_air ? n_water_ : n_air_;

    double theta1_rad = incident_angle_deg * M_PI / 180.0;
    double cos_theta1 = std::cos(theta1_rad);
    double sin_theta1 = std::sin(theta1_rad);

    // Check for total internal reflection
    double sin_theta2 = (n1 / n2) * sin_theta1;
    if (std::abs(sin_theta2) > 1.0) {
        return 1.0;  // Total reflection
    }

    double theta2_rad = std::asin(sin_theta2);
    double cos_theta2 = std::cos(theta2_rad);

    // Fresnel coefficients for s and p polarization
    double rs = (n1 * cos_theta1 - n2 * cos_theta2) /
                (n1 * cos_theta1 + n2 * cos_theta2);
    double rp = (n2 * cos_theta1 - n1 * cos_theta2) /
                (n2 * cos_theta1 + n1 * cos_theta2);

    // Reflectance for unpolarized light
    double R = 0.5 * (rs * rs + rp * rp);

    return R;
}

double AtmosphericOpticalModel::getExtinctionCoefficient(
    double wavelength_nm) const
{
    // Total extinction = Rayleigh + Mie + Absorption

    double beta_ray = computeRayleighScattering(wavelength_nm);
    double beta_mie = computeMieScattering(wavelength_nm);
    double beta_abs = computeMolecularAbsorption(wavelength_nm);

    return beta_ray + beta_mie + beta_abs;
}

double AtmosphericOpticalModel::computeRayleighScattering(
    double wavelength_nm) const
{
    // Rayleigh scattering: β ∝ λ^-4

    double lambda_550 = 550.0;  // Reference wavelength (nm)
    double beta_550 = 0.01;     // Scattering at 550 nm (1/km)

    double ratio = wavelength_nm / lambda_550;
    return beta_550 * std::pow(ratio, -4.0);
}

double AtmosphericOpticalModel::computeMieScattering(double wavelength_nm) const
{
    // Mie scattering (aerosols): β ∝ λ^-α, where α ≈ 1.3

    double lambda_550 = 550.0;
    double visibility_km = atm_props_.visibility_km;

    // Koschmieder equation: visibility = 3.912 / β
    double beta_550 = 3.912 / visibility_km;

    double ratio = wavelength_nm / lambda_550;
    return beta_550 * std::pow(ratio, -1.3);
}

double AtmosphericOpticalModel::computeMolecularAbsorption(
    double wavelength_nm) const
{
    // Simplified absorption bands
    // Strong absorption in UV (<300 nm) and IR (>1000 nm)

    if (wavelength_nm < 300.0) {
        return 0.5;  // Strong UV absorption (O3, O2)
    } else if (wavelength_nm > 700.0 && wavelength_nm < 900.0) {
        return 0.02 * atm_props_.water_vapor_cm;  // Water vapor absorption
    } else if (wavelength_nm > 1300.0) {
        return 0.1 * atm_props_.water_vapor_cm;  // Strong water vapor bands
    }

    return 0.001;  // Minimal absorption in visible
}

// ============================================================================
// UnderwaterOpticalModel Implementation
// ============================================================================

UnderwaterOpticalModel::UnderwaterOpticalModel()
{
    updateCoefficients();
}

double UnderwaterOpticalModel::computeTransmittance(
    double distance_m,
    double wavelength_nm) const
{
    // Beer-Lambert law: T = exp(-c·d)
    double c = getAttenuationCoeff(wavelength_nm);
    return std::exp(-c * distance_m);
}

double UnderwaterOpticalModel::computeVeilingLight(
    double distance_m,
    double wavelength_nm) const
{
    // Veiling light due to backscatter
    // B = B∞ · (1 - exp(-c·d))

    double c = getAttenuationCoeff(wavelength_nm);
    (void)getScatteringCoeff(
        wavelength_nm);  // Could be used for more detailed model

    // Ambient water radiance
    double B_inf = 0.1;  // W/m²/sr (typical shallow water)

    double veiling_light = B_inf * (1.0 - std::exp(-c * distance_m));

    return veiling_light;
}

double UnderwaterOpticalModel::computeContrastTransmittance(
    double distance_m,
    double wavelength_nm) const
{
    // Contrast transmittance: T_C = exp(-c·d)
    // where c is beam attenuation coefficient

    double c = getAttenuationCoeff(wavelength_nm);
    return std::exp(-c * distance_m);
}

double UnderwaterOpticalModel::getMaximumRange(
    double contrast_threshold,
    double wavelength_nm) const
{
    // Maximum visible range based on contrast threshold
    // C(d) = C0 · exp(-c·d)
    // Solve for d when C(d) = C_threshold

    double c = getAttenuationCoeff(wavelength_nm);
    double C0 = 1.0;  // Initial contrast (black target on bright background)

    if (contrast_threshold <= 0.0 || contrast_threshold >= C0) {
        return 0.0;
    }

    double d_max = -std::log(contrast_threshold / C0) / c;

    return d_max;
}

UnderwaterOpticalProperties UnderwaterOpticalModel::createWaterType(
    const std::string& water_type)
{
    UnderwaterOpticalProperties props;
    props.water_type = water_type;

    if (water_type == "clear") {
        // Clear ocean water (oligotrophic)
        props.attenuation_coeff_m = 0.05;
        props.scattering_coeff_m = 0.01;
        props.absorption_coeff_m = 0.04;
        props.turbidity_NTU = 0.5;
    } else if (water_type == "coastal") {
        // Coastal water (mesotrophic)
        props.attenuation_coeff_m = 0.2;
        props.scattering_coeff_m = 0.12;
        props.absorption_coeff_m = 0.08;
        props.turbidity_NTU = 2.0;
    } else if (water_type == "harbor") {
        // Harbor/turbid water (eutrophic)
        props.attenuation_coeff_m = 0.5;
        props.scattering_coeff_m = 0.35;
        props.absorption_coeff_m = 0.15;
        props.turbidity_NTU = 10.0;
    }

    return props;
}

void UnderwaterOpticalModel::updateCoefficients()
{
    // Coefficients are already set in water_props_
    // Could add wavelength-dependent models here
}

double UnderwaterOpticalModel::getAttenuationCoeff(double wavelength_nm) const
{
    // Wavelength-dependent attenuation
    // Blue light (450-500 nm) penetrates deepest
    // Red light (>600 nm) is absorbed quickly

    double c_base = water_props_.attenuation_coeff_m;

    // Wavelength factor
    double factor = 1.0;
    if (wavelength_nm < 500.0) {
        // Blue-green: lower attenuation
        factor = 0.7 + 0.3 * (wavelength_nm - 450.0) / 50.0;
    } else if (wavelength_nm > 600.0) {
        // Red: higher attenuation
        factor = 1.0 + 2.0 * (wavelength_nm - 600.0) / 100.0;
    }

    return c_base * factor;
}

double UnderwaterOpticalModel::getScatteringCoeff(double wavelength_nm) const
{
    // Scattering decreases with wavelength: b ∝ λ^-α

    double b_base = water_props_.scattering_coeff_m;
    double lambda_550 = 550.0;
    double ratio = wavelength_nm / lambda_550;

    // α ≈ 1.0 for underwater scattering
    return b_base * std::pow(ratio, -1.0);
}

// ============================================================================
// OpticalPropagationModel Implementation
// ============================================================================

OpticalPropagationModel::OpticalPropagationModel() {}

double OpticalPropagationModel::computeTransmittance(
    double air_distance_m,
    double water_distance_m,
    double wavelength_nm,
    double air_water_angle_deg) const
{
    // Atmospheric transmittance
    double T_atm =
        atm_model_.computeTransmittance(air_distance_m, wavelength_nm);

    // Interface reflection loss
    double R = atm_model_.computeFresnelReflection(air_water_angle_deg, true);
    double T_interface = 1.0 - R;

    // Underwater transmittance
    double T_water =
        water_model_.computeTransmittance(water_distance_m, wavelength_nm);

    // Total transmittance
    return T_atm * T_interface * T_water;
}

double OpticalPropagationModel::computeBandTransmittance(
    double air_distance_m,
    double water_distance_m,
    OpticalBand band,
    double air_water_angle_deg) const
{
    // Get wavelength range for band
    auto wavelengths = getBandWavelengths(band);
    double lambda_min = wavelengths.first;
    double lambda_max = wavelengths.second;

    // Integrate over band (simple trapezoidal rule with 10 samples)
    int num_samples = 10;
    double T_sum = 0.0;

    for (int i = 0; i < num_samples; ++i) {
        double lambda = lambda_min + (i / double(num_samples - 1)) *
                                         (lambda_max - lambda_min);
        double T = computeTransmittance(
            air_distance_m,
            water_distance_m,
            lambda,
            air_water_angle_deg);
        T_sum += T;
    }

    return T_sum / num_samples;
}

std::pair<double, double> OpticalPropagationModel::getBandWavelengths(
    OpticalBand band)
{
    switch (band) {
        case OpticalBand::VISIBLE:
            return {400.0, 700.0};
        case OpticalBand::NEAR_IR:
            return {700.0, 1400.0};
        case OpticalBand::MID_IR:
            return {3000.0, 5000.0};
        case OpticalBand::LONG_IR:
            return {8000.0, 14000.0};
        default:
            return {400.0, 700.0};
    }
}

}  // namespace environment
}  // namespace lotusim
