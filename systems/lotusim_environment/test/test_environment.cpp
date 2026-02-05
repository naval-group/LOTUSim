/**
 * @file test_environment.cpp
 * @brief Unit tests for LOTUSim environment simulation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include <gtest/gtest.h>

#include "lotusim_environment/ocean/ocean_model.hpp"
#include "lotusim_environment/propagation/acoustic_propagation.hpp"
#include "lotusim_environment/propagation/optical_propagation.hpp"
#include "lotusim_environment/propagation/radar_propagation.hpp"
#include "lotusim_environment/vehicle/operational_effects.hpp"
#include "lotusim_environment/vehicle/seakeeping_model.hpp"
#include "lotusim_environment/weather/sea_state.hpp"
#include "lotusim_environment/weather/wave_model.hpp"
#include "lotusim_environment/weather/weather_conditions.hpp"
#include "lotusim_environment/weather/wind_model.hpp"

using namespace lotusim::environment;

// ============================================================================
// Wind Model Tests (Sprint 9.1.1)
// ============================================================================

TEST(WindModel, Construction)
{
    WindParameters params;
    params.speed_knots = 10.0;
    params.direction_deg = 45.0;

    WindModel wind(params);

    EXPECT_NEAR(wind.getWindSpeed(), 10.0 * 0.514444, 0.1);
    EXPECT_NEAR(wind.getWindDirection(), 45.0, 1.0);
}

TEST(WindModel, BeaufortScale)
{
    WindModel wind;

    // Test Beaufort conversions
    wind.setFromBeaufort(BeaufortScale::CALM, 0.0);
    EXPECT_EQ(wind.getBeaufortScale(), BeaufortScale::CALM);
    EXPECT_LT(wind.getWindSpeed(), 1.0 * 0.514444);

    wind.setFromBeaufort(BeaufortScale::GENTLE_BREEZE, 0.0);
    EXPECT_EQ(wind.getBeaufortScale(), BeaufortScale::GENTLE_BREEZE);

    wind.setFromBeaufort(BeaufortScale::GALE, 0.0);
    EXPECT_EQ(wind.getBeaufortScale(), BeaufortScale::GALE);

    wind.setFromBeaufort(BeaufortScale::HURRICANE, 0.0);
    EXPECT_EQ(wind.getBeaufortScale(), BeaufortScale::HURRICANE);
}

TEST(WindModel, GustModeling)
{
    WindParameters params;
    params.speed_knots = 20.0;
    params.gust_factor = 1.5;
    params.gust_period_s = 10.0;
    params.turbulence_intensity = 0.0;  // No turbulence for this test

    WindModel wind(params);

    double base_speed = wind.getWindSpeed();

    // Simulate through half gust period
    wind.update(5.0);  // Peak gust
    double peak_speed = wind.getWindSpeed();

    // Gust should increase speed
    EXPECT_GT(peak_speed, base_speed * 1.2);
}

TEST(WindModel, DirectionPersistence)
{
    WindParameters params;
    params.speed_knots = 15.0;
    params.direction_deg = 90.0;  // East
    params.turbulence_intensity = 0.0;

    WindModel wind(params);

    // Direction should remain approximately eastward despite gusts
    wind.update(10.0);
    double dir = wind.getWindDirection();

    EXPECT_NEAR(dir, 90.0, 5.0);  // Within 5 degrees
}

TEST(WindModel, KnotsConversion)
{
    double knots = 10.0;
    double mps = knotsToMps(knots);
    double back_to_knots = mpsToKnots(mps);

    EXPECT_NEAR(mps, 5.14444, 0.001);
    EXPECT_NEAR(back_to_knots, knots, 0.001);
}

// ============================================================================
// Wave Model Tests (Sprint 9.1.1)
// ============================================================================

TEST(WaveModel, Construction)
{
    WaveParameters params;
    params.significant_wave_height_m = 2.0;
    params.peak_period_s = 8.0;

    WaveModel waves(params);

    EXPECT_DOUBLE_EQ(waves.getSignificantWaveHeight(), 2.0);
    EXPECT_DOUBLE_EQ(waves.getPeakPeriod(), 8.0);
}

TEST(WaveModel, ElevationVariation)
{
    WaveParameters params;
    params.significant_wave_height_m = 3.0;
    params.peak_period_s = 10.0;

    WaveModel waves(params);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);

    // Get elevation at t=0
    double eta0 = waves.getElevation(pos);

    // Update time
    waves.update(2.5);  // Quarter period

    // Get elevation at t=2.5s
    double eta1 = waves.getElevation(pos);

    // Elevation should change over time
    EXPECT_NE(eta0, eta1);
}

TEST(WaveModel, SpatialVariation)
{
    WaveParameters params;
    params.significant_wave_height_m = 2.5;
    params.peak_period_s = 8.0;

    WaveModel waves(params);

    Eigen::Vector3d pos1(0.0, 0.0, 0.0);
    Eigen::Vector3d pos2(100.0, 0.0, 0.0);

    double eta1 = waves.getElevation(pos1);
    double eta2 = waves.getElevation(pos2);

    // Elevation should vary spatially
    EXPECT_NE(eta1, eta2);
}

TEST(WaveModel, OrbitalVelocity)
{
    WaveParameters params;
    params.significant_wave_height_m = 2.0;
    params.peak_period_s = 8.0;

    WaveModel waves(params);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);

    // Surface velocity
    Eigen::Vector3d vel_surface = waves.getOrbitalVelocity(pos, 0.0);

    // Deep velocity
    Eigen::Vector3d vel_deep = waves.getOrbitalVelocity(pos, 50.0);

    // Orbital velocity should decay with depth
    EXPECT_GT(vel_surface.norm(), vel_deep.norm());
}

TEST(WaveModel, PiersonMoskowitzSpectrum)
{
    WaveParameters params;
    params.spectrum = WaveSpectrumType::PIERSON_MOSKOWITZ;
    params.significant_wave_height_m = 3.0;
    params.peak_period_s = 10.0;

    WaveModel waves(params);

    // Should have generated components
    EXPECT_GT(waves.getComponents().size(), 0);
}

TEST(WaveModel, JONSWAPSpectrum)
{
    WaveParameters params;
    params.spectrum = WaveSpectrumType::JONSWAP;
    params.significant_wave_height_m = 2.5;
    params.peak_period_s = 9.0;
    params.gamma = 3.3;

    WaveModel waves(params);

    // Should have generated components
    EXPECT_GT(waves.getComponents().size(), 0);
}

TEST(WaveModel, SetFromWind)
{
    WaveModel waves;

    // Test fully developed sea (large fetch)
    waves.setFromWind(25.0, 1000.0);  // 25 knots, 1000 km fetch

    EXPECT_GT(waves.getSignificantWaveHeight(), 0.0);
    EXPECT_GT(waves.getPeakPeriod(), 0.0);

    // Test fetch-limited sea
    waves.setFromWind(20.0, 100.0);  // 20 knots, 100 km fetch

    EXPECT_GT(waves.getSignificantWaveHeight(), 0.0);
    EXPECT_GT(waves.getPeakPeriod(), 0.0);
}

TEST(WaveModel, ComponentGeneration)
{
    WaveParameters params;
    params.significant_wave_height_m = 2.0;
    params.peak_period_s = 8.0;

    WaveModel waves(params);

    const auto& components = waves.getComponents();

    // Should have 50 components (as per implementation)
    EXPECT_EQ(components.size(), 50);

    // All components should have valid properties
    for (const auto& comp : components) {
        EXPECT_GT(comp.amplitude, 0.0);
        EXPECT_GT(comp.frequency, 0.0);
        EXPECT_GT(comp.wave_number, 0.0);
    }
}

TEST(WaveModel, TimeEvolution)
{
    WaveParameters params;
    params.significant_wave_height_m = 2.0;
    params.peak_period_s = 8.0;

    WaveModel waves(params);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);

    // Record elevations over time
    std::vector<double> elevations;
    for (int i = 0; i < 10; ++i) {
        elevations.push_back(waves.getElevation(pos));
        waves.update(1.0);
    }

    // Elevations should vary over time (not all the same)
    bool has_variation = false;
    for (size_t i = 1; i < elevations.size(); ++i) {
        if (std::abs(elevations[i] - elevations[0]) > 0.01) {
            has_variation = true;
            break;
        }
    }
    EXPECT_TRUE(has_variation);
}

// ============================================================================
// Sea State Tests (Sprint 9.1.2)
// ============================================================================

TEST(SeaState, Construction)
{
    SeaState sea;
    EXPECT_GT(sea.getSignificantWaveHeight(), 0.0);
}

TEST(SeaState, DouglasScale)
{
    SeaState sea;

    sea.setFromWaveHeight(0.0);
    EXPECT_EQ(sea.getDouglasScale(), DouglasSeaState::CALM_GLASSY);

    sea.setFromWaveHeight(1.0);
    EXPECT_EQ(sea.getDouglasScale(), DouglasSeaState::SLIGHT);

    sea.setFromWaveHeight(3.0);
    EXPECT_EQ(sea.getDouglasScale(), DouglasSeaState::ROUGH);

    sea.setFromWaveHeight(10.0);
    EXPECT_EQ(sea.getDouglasScale(), DouglasSeaState::VERY_HIGH);
}

TEST(SeaState, SetFromWindSpeed)
{
    SeaState sea;
    sea.setFromWindSpeed(25.0);  // 25 knots

    EXPECT_GT(sea.getSignificantWaveHeight(), 0.0);
    EXPECT_DOUBLE_EQ(sea.getWindSpeed(), 25.0);
}

TEST(SeaState, Description)
{
    SeaState sea;
    sea.setFromWaveHeight(3.0);
    std::string desc = sea.getDescription();
    EXPECT_FALSE(desc.empty());
}

// ============================================================================
// Weather Conditions Tests (Sprint 9.1.3)
// ============================================================================

TEST(WeatherModel, Construction)
{
    WeatherModel weather;
    EXPECT_GT(weather.getVisibility(), 0.0);
}

TEST(WeatherModel, VisibilityCategory)
{
    WeatherModel weather;

    WeatherConditions cond;
    cond.visibility_km = 0.1;
    weather.setConditions(cond);
    EXPECT_EQ(weather.getVisibilityCategory(), VisibilityCategory::DENSE_FOG);

    cond.visibility_km = 3.0;
    weather.setConditions(cond);
    EXPECT_EQ(weather.getVisibilityCategory(), VisibilityCategory::MODERATE);

    cond.visibility_km = 15.0;
    weather.setConditions(cond);
    EXPECT_EQ(weather.getVisibilityCategory(), VisibilityCategory::EXCELLENT);
}

TEST(WeatherModel, RainAttenuation)
{
    WeatherModel weather;

    WeatherConditions cond;
    cond.precipitation_type = PrecipitationType::RAIN;
    cond.precipitation_rate_mm_h = 10.0;  // Moderate rain
    weather.setConditions(cond);

    double attenuation = weather.getRadarAttenuation_dB();
    EXPECT_GT(attenuation, 0.0);
}

TEST(WeatherModel, OpticalAttenuation)
{
    WeatherModel weather;

    WeatherConditions cond;
    cond.visibility_km = 1.0;  // Fog
    weather.setConditions(cond);

    double attenuation = weather.getOpticalAttenuation();
    EXPECT_GT(attenuation, 0.0);
    EXPECT_LT(attenuation, 1.0);
}

TEST(WeatherModel, Description)
{
    WeatherModel weather;

    WeatherConditions cond;
    cond.visibility_km = 8.0;
    cond.precipitation_type = PrecipitationType::RAIN;
    cond.precipitation_rate_mm_h = 5.0;
    weather.setConditions(cond);

    std::string desc = weather.getDescription();
    EXPECT_FALSE(desc.empty());
}

// ============================================================================
// Ocean Tests (WP9.2)
// ============================================================================

TEST(OceanCurrent, Construction)
{
    OceanCurrentModel current;
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    Eigen::Vector3d vel = current.getCurrentVelocity(pos, 0.0);
    EXPECT_GT(vel.norm(), 0.0);
}

TEST(OceanCurrent, DepthDecay)
{
    OceanCurrentModel current;
    Eigen::Vector3d pos(0.0, 0.0, 0.0);

    Eigen::Vector3d vel_surface = current.getCurrentVelocity(pos, 0.0);
    Eigen::Vector3d vel_deep = current.getCurrentVelocity(pos, 100.0);

    EXPECT_GT(vel_surface.norm(), vel_deep.norm());
}

TEST(ThermalProfile, DefaultProfile)
{
    ThermalProfileModel thermal;

    double temp_surface = thermal.getTemperature(0.0);
    double temp_deep = thermal.getTemperature(1000.0);

    EXPECT_GT(temp_surface, temp_deep);
}

TEST(ThermalProfile, SoundSpeed)
{
    ThermalProfileModel thermal;

    double c = thermal.getSoundSpeed(0.0);
    EXPECT_GT(c, 1400.0);
    EXPECT_LT(c, 1600.0);
}

TEST(ThermalProfile, ThermoclineDetection)
{
    ThermalProfileModel thermal;
    double thermocline = thermal.getThermoclineDepth();
    EXPECT_GT(thermocline, 0.0);
    EXPECT_LT(thermocline, 200.0);
}

TEST(Bathymetry, Construction)
{
    BathymetryModel bathy;
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double depth = bathy.getDepth(pos);
    EXPECT_GT(depth, 0.0);
}

TEST(Bathymetry, FlatBottom)
{
    BathymetryModel bathy;
    bathy.setFlatBottom(200.0);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(bathy.getDepth(pos), 200.0);
}

TEST(Bathymetry, DeepWater)
{
    BathymetryModel bathy;
    bathy.setFlatBottom(100.0);

    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    EXPECT_TRUE(bathy.isDeepWater(pos));
}

// ============================================================================
// Propagation Tests (WP9.3)
// ============================================================================

TEST(RadarPropagation, DirectPath)
{
    RadarPropagationModel radar;

    Eigen::Vector3d tx_pos(0.0, 0.0, -10.0);     // 10m altitude
    Eigen::Vector3d rx_pos(1000.0, 0.0, -20.0);  // 20m altitude, 1km away
    double frequency = 10e9;                     // 10 GHz (X-band)

    auto paths = radar.computePaths(tx_pos, rx_pos, frequency);

    EXPECT_GT(paths.size(), 0);
    EXPECT_GT(paths[0].total_path_loss_dB, 0.0);
}

TEST(RadarPropagation, RadarHorizon)
{
    RadarPropagationModel radar;

    double tx_alt = 10.0;  // 10m
    double rx_alt = 20.0;  // 20m
    double horizon = radar.getRadarHorizon(tx_alt, rx_alt);

    // Should be around 31 km for these altitudes with k=4/3
    EXPECT_GT(horizon, 25000.0);
    EXPECT_LT(horizon, 35000.0);
}

TEST(RadarPropagation, AtmosphericDuct)
{
    RadarPropagationModel radar;

    AtmosphericDuct duct;
    duct.base_altitude_m = 0.0;
    duct.top_altitude_m = 50.0;
    duct.strength = 1.0;
    duct.is_surface_duct = true;

    radar.setDuct(duct);

    EXPECT_TRUE(radar.isInDuct(25.0));
    EXPECT_FALSE(radar.isInDuct(100.0));
}

TEST(RadarPropagation, FreeSpaceLoss)
{
    RadarPropagationModel radar;

    Eigen::Vector3d tx_pos(0.0, 0.0, -10.0);
    Eigen::Vector3d rx_pos(1000.0, 0.0, -10.0);  // 1km range
    double frequency = 10e9;

    auto paths = radar.computePaths(tx_pos, rx_pos, frequency);

    // Free space path loss at 1km, 10 GHz ~ 112 dB (includes atmospheric
    // absorption)
    EXPECT_GT(paths[0].total_path_loss_dB, 100.0);
    EXPECT_LT(paths[0].total_path_loss_dB, 120.0);
}

TEST(MultipathModel, FadingCalculation)
{
    std::vector<PropagationPath> paths(2);

    paths[0].total_path_loss_dB = 90.0;
    paths[0].total_distance_m = 1000.0;

    paths[1].total_path_loss_dB = 95.0;
    paths[1].total_distance_m = 1050.0;  // Slightly longer path

    double fading = MultipathModel::computeMultipathFading(paths, 10e9);

    // Fading can be positive or negative
    EXPECT_LT(std::abs(fading), 20.0);
}

TEST(AcousticPropagation, TransmissionLoss)
{
    AcousticPropagationModel acoustic;

    Eigen::Vector3d source(0.0, 0.0, -100.0);       // 100m depth
    Eigen::Vector3d receiver(1000.0, 0.0, -100.0);  // 1km away
    double frequency = 5000.0;                      // 5 kHz

    double TL = acoustic.computeTransmissionLoss(source, receiver, frequency);

    EXPECT_GT(TL, 50.0);  // Should have significant loss
    EXPECT_LT(TL, 100.0);
}

TEST(AcousticPropagation, SofarChannel)
{
    ThermalProfile profile = ThermalProfileModel::createDefaultProfile();
    AcousticPropagationModel acoustic(&profile);

    double axis_depth = acoustic.getSofarAxisDepth();

    EXPECT_GE(axis_depth, 500.0);
    EXPECT_LT(axis_depth, 1500.0);
}

TEST(AcousticPropagation, RayTracing)
{
    ThermalProfile profile = ThermalProfileModel::createDefaultProfile();
    AcousticPropagationModel acoustic(&profile);

    Eigen::Vector3d source(0.0, 0.0, -100.0);
    Eigen::Vector3d receiver(5000.0, 0.0, -100.0);  // 5km away
    double frequency = 3000.0;                      // 3 kHz
    double water_depth = 1000.0;

    auto result = acoustic.traceRays(source, receiver, frequency, water_depth);

    // May or may not find eigenrays depending on environment
    EXPECT_GT(result.transmission_loss_dB, 0.0);
}

TEST(AbsorptionModel, FrancoisGarrison)
{
    double alpha = AbsorptionModel::computeAbsorption(
        5000.0,  // 5 kHz
        15.0,    // 15°C
        35.0,    // 35 ppt salinity
        100.0,   // 100m depth
        8.0);    // pH 8

    // Absorption at 5 kHz should be around 0.5-1.5 dB/km
    EXPECT_GT(alpha, 0.1);
    EXPECT_LT(alpha, 3.0);
}

TEST(OpticalPropagation, AtmosphericTransmittance)
{
    AtmosphericOpticalModel atm;

    // Set good visibility
    AtmosphericOpticalProperties props;
    props.visibility_km = 20.0;  // 20 km visibility
    atm.setAtmosphericProperties(props);

    double distance = 1000.0;   // 1 km
    double wavelength = 550.0;  // Green light (nm)

    double T = atm.computeTransmittance(distance, wavelength);

    EXPECT_GT(T, 0.7);  // Should have good transmission
    EXPECT_LT(T, 1.0);
}

TEST(OpticalPropagation, Refraction)
{
    AtmosphericOpticalModel atm;

    // Light going from air to water at 45 degrees
    double incident_angle = 45.0;
    double refracted_angle = atm.applyRefraction(incident_angle, true);

    // Should bend toward normal (smaller angle)
    EXPECT_LT(refracted_angle, incident_angle);
    EXPECT_GT(refracted_angle, 0.0);
}

TEST(OpticalPropagation, FresnelReflection)
{
    AtmosphericOpticalModel atm;

    // Normal incidence (0 degrees)
    double R_normal = atm.computeFresnelReflection(0.0, true);

    // Grazing incidence (85 degrees)
    double R_grazing = atm.computeFresnelReflection(85.0, true);

    // Reflection increases with angle
    EXPECT_GT(R_grazing, R_normal);
    EXPECT_LT(R_normal, 0.1);
    EXPECT_GT(R_grazing, 0.5);
}

TEST(OpticalPropagation, UnderwaterTransmittance)
{
    UnderwaterOpticalModel water;

    double distance = 10.0;     // 10 m
    double wavelength = 500.0;  // Blue-green light (nm)

    double T = water.computeTransmittance(distance, wavelength);

    EXPECT_GT(T, 0.0);
    EXPECT_LT(T, 1.0);
}

TEST(OpticalPropagation, WaterTypes)
{
    auto clear = UnderwaterOpticalModel::createWaterType("clear");
    auto coastal = UnderwaterOpticalModel::createWaterType("coastal");
    auto harbor = UnderwaterOpticalModel::createWaterType("harbor");

    // Clear water should have lowest attenuation
    EXPECT_LT(clear.attenuation_coeff_m, coastal.attenuation_coeff_m);
    EXPECT_LT(coastal.attenuation_coeff_m, harbor.attenuation_coeff_m);
}

TEST(OpticalPropagation, MaximumRange)
{
    UnderwaterOpticalModel water;

    double contrast_threshold = 0.02;  // 2% contrast
    double wavelength = 500.0;         // Blue-green

    double max_range = water.getMaximumRange(contrast_threshold, wavelength);

    EXPECT_GT(max_range, 10.0);   // Should see at least 10m
    EXPECT_LT(max_range, 200.0);  // But not too far
}

TEST(OpticalPropagation, CombinedAirWater)
{
    OpticalPropagationModel combined;

    double air_dist = 100.0;    // 100m through air
    double water_dist = 10.0;   // 10m through water
    double wavelength = 550.0;  // Green light
    double angle = 30.0;        // 30 degrees from normal

    double T =
        combined.computeTransmittance(air_dist, water_dist, wavelength, angle);

    EXPECT_GT(T, 0.0);
    EXPECT_LT(T, 1.0);
}

// ============================================================================
// Seakeeping Model Tests (Sprint 9.4.1)
// ============================================================================

TEST(SeakeepingModel, Construction)
{
    SeakeepingModel seakeeping;

    // Should have default RAOs for standard vessels
    const RAOData& frigate_rao = seakeeping.getRAO("frigate");
    EXPECT_GT(frigate_rao.frequencies_rad_s.size(), 0);
    EXPECT_GT(frigate_rao.headings_deg.size(), 0);
}

TEST(SeakeepingModel, StandardRAO)
{
    RAOData rao = SeakeepingModel::createStandardRAO("frigate");

    // Check structure
    EXPECT_GT(rao.frequencies_rad_s.size(), 5);
    EXPECT_GT(rao.headings_deg.size(), 5);
    EXPECT_TRUE(rao.amplitudes.find(MotionType::HEAVE) != rao.amplitudes.end());
    EXPECT_TRUE(rao.amplitudes.find(MotionType::ROLL) != rao.amplitudes.end());
    EXPECT_TRUE(rao.amplitudes.find(MotionType::PITCH) != rao.amplitudes.end());

    // Heave RAO should be positive
    for (const auto& freq_data : rao.amplitudes.at(MotionType::HEAVE)) {
        for (double amp : freq_data) {
            EXPECT_GE(amp, 0.0);
        }
    }
}

TEST(SeakeepingModel, ComputeMotions)
{
    SeakeepingModel seakeeping;

    // Moderate sea state (Hs = 2m, Tp = 7s)
    double significant_wave_height = 2.0;
    double peak_period = 7.0;
    double wave_heading = 0.0;  // Head seas
    double ship_speed = 10.0;   // 10 m/s

    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        significant_wave_height,
        peak_period,
        wave_heading,
        ship_speed);

    // Motions should be reasonable
    EXPECT_GT(std::abs(motions.heave), 0.0);
    EXPECT_LT(std::abs(motions.heave), 5.0);  // < 5m heave

    EXPECT_GT(std::abs(motions.pitch), 0.0);
    EXPECT_LT(std::abs(motions.pitch), 0.3);  // < ~17 degrees

    // Roll should be small in head seas
    EXPECT_LT(std::abs(motions.roll), 0.2);
}

TEST(SeakeepingModel, BeamSeasRoll)
{
    SeakeepingModel seakeeping;

    // Beam seas (90 degrees)
    ShipMotions motions =
        seakeeping.computeMotions("frigate", 2.0, 7.0, 90.0, 10.0);

    // Roll should be non-zero in beam seas (even if small)
    EXPECT_GT(std::abs(motions.roll), 0.001);
}

TEST(SeakeepingModel, LocationAcceleration)
{
    SeakeepingModel seakeeping;

    ShipMotions motions;
    motions.heave_acceleration = 2.0;  // 2 m/s²
    motions.roll = 0.1;                // ~6 degrees
    motions.roll_velocity = 0.05;
    motions.roll_acceleration = 0.1;

    // Location at bow, 50m forward
    Eigen::Vector3d position(50.0, 0.0, 0.0);

    LocationAcceleration accel =
        seakeeping.computeLocationAcceleration(motions, position);

    EXPECT_GT(accel.total_magnitude, 0.0);
    EXPECT_LT(accel.total_magnitude, 10.0);  // Reasonable acceleration
}

TEST(SeakeepingModel, MSI)
{
    SeakeepingModel seakeeping;

    ShipMotions motions;
    motions.heave_acceleration = 1.0;  // 1 m/s² vertical acceleration

    // Short exposure
    double msi_1h = seakeeping.computeMSI(motions, 1.0);
    EXPECT_GE(msi_1h, 0.0);
    EXPECT_LE(msi_1h, 100.0);

    // Longer exposure should have higher MSI
    double msi_4h = seakeeping.computeMSI(motions, 4.0);
    EXPECT_GT(msi_4h, msi_1h);
}

TEST(SeakeepingModel, OperabilityCriteria)
{
    OperabilityCriteria criteria =
        SeakeepingModel::getStandardCriteria("frigate");

    EXPECT_GT(criteria.max_roll_deg, 10.0);
    EXPECT_GT(criteria.max_pitch_deg, 5.0);
    EXPECT_GT(criteria.max_sea_state, 4);
}

TEST(SeakeepingModel, OperabilityAssessment)
{
    SeakeepingModel seakeeping;

    ShipMotions motions;
    motions.heave = 0.5;
    motions.roll = 0.08;   // ~5 degrees
    motions.pitch = 0.04;  // ~2 degrees
    motions.heave_acceleration = 0.2;

    OperabilityCriteria criteria =
        SeakeepingModel::getStandardCriteria("frigate");

    OperabilityStatus status =
        seakeeping.assessOperability(motions, criteria, 2, 10.0);

    // Assessment should complete and return valid values
    EXPECT_GE(status.overall_score, 0.0);
    EXPECT_LE(status.overall_score, 1.0);
    EXPECT_GE(status.msi_percent, 0.0);
    EXPECT_LE(status.msi_percent, 100.0);
}

TEST(SeakeepingModel, OperabilityExcessive)
{
    SeakeepingModel seakeeping;

    ShipMotions motions;
    motions.heave = 5.0;  // Excessive heave
    motions.roll = 0.4;   // ~23 degrees - excessive
    motions.pitch = 0.2;  // ~11 degrees - excessive
    motions.heave_acceleration = 5.0;

    OperabilityCriteria criteria =
        SeakeepingModel::getStandardCriteria("frigate");

    OperabilityStatus status =
        seakeeping.assessOperability(motions, criteria, 7, 30.0);

    // Should NOT be operable in severe conditions
    EXPECT_FALSE(status.is_operable);
    EXPECT_GT(status.limiting_factors.size(), 0);
    EXPECT_LT(status.overall_score, 0.7);
}

// ============================================================================
// Operational Effects Tests (Sprint 9.4.2)
// ============================================================================

TEST(OperationalEffects, Construction)
{
    OperationalEffectsModel ops_effects;

    // Should construct successfully
    EXPECT_TRUE(true);
}

TEST(OperationalEffects, RadarPerformanceGood)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 2;  // Slight
    conditions.precipitation_mm_h = 0.0;
    conditions.wind_speed_m_s = 10.0;

    ShipMotions motions;
    motions.roll = 0.05;  // ~3 degrees

    SensorPerformance perf =
        ops_effects.computeRadarPerformance(conditions, motions);

    // Good conditions - minimal degradation
    EXPECT_GT(perf.detection_range_factor, 0.85);
    EXPECT_EQ(perf.availability, 1.0);
}

TEST(OperationalEffects, RadarPerformanceDegraded)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 6;              // Very rough
    conditions.precipitation_mm_h = 30.0;  // Heavy rain
    conditions.wind_speed_m_s = 25.0;

    ShipMotions motions;
    motions.roll = 0.2;  // ~11 degrees

    SensorPerformance perf =
        ops_effects.computeRadarPerformance(conditions, motions);

    // Degraded conditions
    EXPECT_LT(perf.detection_range_factor, 0.85);
    EXPECT_GT(perf.degradation_reasons.size(), 0);
}

TEST(OperationalEffects, SonarPerformance)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 3;
    conditions.ship_speed_m_s = 5.0;  // Low speed - good for sonar

    ShipMotions motions;
    motions.roll = 0.05;

    SensorPerformance perf =
        ops_effects.computeSonarPerformance(conditions, motions);

    EXPECT_GT(perf.detection_range_factor, 0.7);
    EXPECT_EQ(perf.availability, 1.0);
}

TEST(OperationalEffects, SonarSelfNoise)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 3;
    conditions.ship_speed_m_s = 15.0;  // High speed - self-noise

    ShipMotions motions;

    SensorPerformance perf =
        ops_effects.computeSonarPerformance(conditions, motions);

    // Self-noise should degrade performance
    EXPECT_LT(perf.detection_range_factor, 1.0);
    EXPECT_GT(perf.degradation_reasons.size(), 0);
}

TEST(OperationalEffects, OptronicsVisibility)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.visibility_km = 2.0;  // Poor visibility
    conditions.precipitation_mm_h = 5.0;

    ShipMotions motions;

    SensorPerformance perf =
        ops_effects.computeOptronicsPerformance(conditions, motions);

    // Poor visibility should degrade optical sensors
    EXPECT_LT(perf.detection_range_factor, 0.7);
    EXPECT_GT(perf.degradation_reasons.size(), 0);
}

TEST(OperationalEffects, MissilePerformanceGood)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 3;
    conditions.wind_speed_m_s = 10.0;

    ShipMotions motions;
    motions.roll = 0.1;    // ~6 degrees
    motions.pitch = 0.05;  // ~3 degrees

    WeaponPerformance perf =
        ops_effects.computeMissilePerformance(conditions, motions);

    EXPECT_TRUE(perf.can_launch);
    EXPECT_GT(perf.max_range_factor, 0.85);
}

TEST(OperationalEffects, MissilePerformanceExcessiveMotion)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 6;

    ShipMotions motions;
    motions.roll = 0.3;    // ~17 degrees - excessive
    motions.pitch = 0.15;  // ~9 degrees - excessive

    WeaponPerformance perf =
        ops_effects.computeMissilePerformance(conditions, motions);

    EXPECT_FALSE(perf.can_launch);
    EXPECT_GT(perf.limitation_reasons.size(), 0);
}

TEST(OperationalEffects, GunPerformance)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    ShipMotions motions;
    motions.roll = 0.15;   // ~9 degrees
    motions.pitch = 0.08;  // ~5 degrees

    WeaponPerformance perf =
        ops_effects.computeGunPerformance(conditions, motions);

    // Should still be able to fire, but with reduced accuracy
    EXPECT_TRUE(perf.can_launch);
    EXPECT_GT(perf.accuracy_factor, 1.0);  // Worse accuracy
}

TEST(OperationalEffects, AircraftOperationsGood)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 2;  // Calmer conditions
    conditions.wind_speed_m_s = 12.0;

    ShipMotions motions;
    motions.heave = 0.3;    // Lower heave
    motions.pitch = 0.015;  // Lower pitch (~1 degree)

    AircraftOperations ops =
        ops_effects.computeAircraftOperations(conditions, motions);

    EXPECT_TRUE(ops.helicopter_launch);
    EXPECT_TRUE(ops.helicopter_recovery);
}

TEST(OperationalEffects, AircraftOperationsLimited)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 6;
    conditions.wind_speed_m_s = 30.0;

    ShipMotions motions;
    motions.heave = 3.0;
    motions.pitch = 0.15;

    AircraftOperations ops =
        ops_effects.computeAircraftOperations(conditions, motions);

    EXPECT_FALSE(ops.helicopter_launch);
    EXPECT_FALSE(ops.helicopter_recovery);
    EXPECT_GT(ops.limitation_reasons.size(), 0);
}

TEST(OperationalEffects, SpeedLimitations)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    ShipMotions motions;
    motions.heave_acceleration = 1.5;

    // Moderate sea state
    SpeedLimitations limits =
        ops_effects.computeSpeedLimitations(conditions, motions, 4);

    EXPECT_GT(limits.max_safe_speed_m_s, 15.0);
    EXPECT_TRUE(limits.full_speed_available);
}

TEST(OperationalEffects, SpeedLimitationsSevere)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    ShipMotions motions;
    motions.heave = 4.0;
    motions.pitch = 0.15;
    motions.heave_acceleration = 4.0;

    // High sea state
    SpeedLimitations limits =
        ops_effects.computeSpeedLimitations(conditions, motions, 7);

    EXPECT_LT(limits.max_safe_speed_m_s, 15.0);
    EXPECT_FALSE(limits.full_speed_available);
    EXPECT_GT(limits.limitation_reasons.size(), 0);
}

TEST(OperationalEffects, CompleteEffects)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions conditions;
    conditions.sea_state = 4;
    conditions.wind_speed_m_s = 18.0;
    conditions.visibility_km = 5.0;
    conditions.ship_speed_m_s = 12.0;

    ShipMotions motions;
    motions.heave = 1.5;
    motions.roll = 0.12;
    motions.pitch = 0.06;
    motions.heave_acceleration = 2.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // All systems should have some performance data
    EXPECT_GT(limits.radar_performance.detection_range_factor, 0.0);
    EXPECT_GT(limits.sonar_performance.detection_range_factor, 0.0);
    EXPECT_GT(limits.overall_operational_readiness, 0.0);
    EXPECT_LE(limits.overall_operational_readiness, 1.0);
}

TEST(OperationalEffects, OverallReadiness)
{
    OperationalEffectsModel ops_effects;

    EnvironmentalConditions good_conditions;
    good_conditions.sea_state = 2;
    good_conditions.wind_speed_m_s = 10.0;

    ShipMotions calm_motions;
    calm_motions.heave = 0.5;
    calm_motions.roll = 0.05;

    OperationalLimitations good_limits =
        ops_effects.computeEffects(good_conditions, calm_motions);

    // Good conditions should have high readiness
    EXPECT_GT(good_limits.overall_operational_readiness, 0.85);

    // Severe conditions
    EnvironmentalConditions severe_conditions;
    severe_conditions.sea_state = 7;
    severe_conditions.wind_speed_m_s = 30.0;

    ShipMotions severe_motions;
    severe_motions.heave = 4.0;
    severe_motions.roll = 0.3;

    OperationalLimitations severe_limits =
        ops_effects.computeEffects(severe_conditions, severe_motions);

    // Severe conditions should have lower readiness
    EXPECT_LT(
        severe_limits.overall_operational_readiness,
        good_limits.overall_operational_readiness);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
