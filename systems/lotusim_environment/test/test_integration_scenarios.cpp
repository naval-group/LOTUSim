/**
 * @file test_integration_scenarios.cpp
 * @brief Integration tests for environmental scenarios (Sprint 9.5.1)
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

/**
 * @brief Complete environmental scenario
 */
struct EnvironmentalScenario {
    std::string name;
    std::string description;

    // Weather conditions
    WindParameters wind;
    int sea_state;
    double visibility_km;
    double precipitation_mm_h;

    // Ocean conditions
    double water_temp_C;
    double salinity_ppt;
    double water_depth_m;

    // Expected outcomes
    bool should_be_operable;
    double min_operational_readiness;
};

// ============================================================================
// Integration Scenario Tests
// ============================================================================

TEST(EnvironmentalScenarios, CalmConditions)
{
    // Scenario: Calm Mediterranean summer day
    EnvironmentalScenario scenario;
    scenario.name = "Calm Mediterranean Summer";
    scenario.description = "Ideal conditions for all operations";
    scenario.wind.speed_knots = 5.0;
    scenario.wind.direction_deg = 0.0;
    scenario.sea_state = 1;
    scenario.visibility_km = 20.0;
    scenario.precipitation_mm_h = 0.0;
    scenario.water_temp_C = 24.0;
    scenario.salinity_ppt = 38.0;
    scenario.water_depth_m = 1000.0;
    scenario.should_be_operable = true;
    scenario.min_operational_readiness = 0.9;

    // Initialize weather models
    WindModel wind(scenario.wind);

    // Setup wave model from sea state
    WaveParameters wave_params;
    wave_params.significant_wave_height_m =
        SeaState::douglasToWaveHeight(DouglasSeaState::CALM_RIPPLED);
    wave_params.peak_period_s = 5.0;
    WaveModel waves(wave_params);

    WeatherConditions weather_cond;
    weather_cond.visibility_km = scenario.visibility_km;
    weather_cond.precipitation_rate_mm_h = scenario.precipitation_mm_h;
    WeatherModel weather(weather_cond);

    // Initialize seakeeping
    SeakeepingModel seakeeping;
    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        waves.getSignificantWaveHeight(),
        waves.getPeakPeriod(),
        0.0,
        10.0);

    // Initialize operational effects
    OperationalEffectsModel ops_effects;
    EnvironmentalConditions conditions;
    conditions.wind_speed_m_s = wind.getWindSpeed();
    conditions.sea_state = scenario.sea_state;
    conditions.visibility_km = scenario.visibility_km;
    conditions.precipitation_mm_h = scenario.precipitation_mm_h;
    conditions.water_temperature_C = scenario.water_temp_C;
    conditions.salinity_ppt = scenario.salinity_ppt;
    conditions.ship_speed_m_s = 10.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // Verify expectations
    EXPECT_GT(
        limits.overall_operational_readiness,
        scenario.min_operational_readiness);
    EXPECT_TRUE(limits.radar_performance.availability > 0.9);
    EXPECT_TRUE(limits.sonar_performance.availability > 0.9);
    EXPECT_TRUE(limits.aircraft_ops.helicopter_launch);
    EXPECT_TRUE(limits.aircraft_ops.helicopter_recovery);
    EXPECT_TRUE(limits.speed_limits.full_speed_available);
}

TEST(EnvironmentalScenarios, ModerateAtlanticConditions)
{
    // Scenario: Moderate Atlantic winter conditions
    EnvironmentalScenario scenario;
    scenario.name = "Moderate Atlantic Winter";
    scenario.description = "Challenging but operable conditions";
    scenario.wind.speed_knots = 20.0;
    scenario.wind.direction_deg = 45.0;
    scenario.sea_state = 4;
    scenario.visibility_km = 8.0;
    scenario.precipitation_mm_h = 2.0;
    scenario.water_temp_C = 12.0;
    scenario.salinity_ppt = 35.0;
    scenario.water_depth_m = 2000.0;
    scenario.should_be_operable = true;
    scenario.min_operational_readiness = 0.6;

    WindModel wind(scenario.wind);

    WaveParameters wave_params;
    wave_params.significant_wave_height_m =
        SeaState::douglasToWaveHeight(DouglasSeaState::MODERATE);
    wave_params.peak_period_s = 7.0;
    WaveModel waves(wave_params);

    SeakeepingModel seakeeping;
    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        waves.getSignificantWaveHeight(),
        waves.getPeakPeriod(),
        45.0,
        12.0);

    OperationalEffectsModel ops_effects;
    EnvironmentalConditions conditions;
    conditions.wind_speed_m_s = wind.getWindSpeed();
    conditions.sea_state = scenario.sea_state;
    conditions.visibility_km = scenario.visibility_km;
    conditions.precipitation_mm_h = scenario.precipitation_mm_h;
    conditions.water_temperature_C = scenario.water_temp_C;
    conditions.salinity_ppt = scenario.salinity_ppt;
    conditions.ship_speed_m_s = 12.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // In moderate conditions, some degradation expected
    EXPECT_GT(
        limits.overall_operational_readiness,
        scenario.min_operational_readiness);
    EXPECT_GT(limits.radar_performance.detection_range_factor, 0.7);
    EXPECT_GT(limits.sonar_performance.detection_range_factor, 0.7);

    // Some systems should show degradation
    EXPECT_GT(limits.radar_performance.degradation_reasons.size(), 0);
}

TEST(EnvironmentalScenarios, SevereStormConditions)
{
    // Scenario: Severe North Atlantic storm
    EnvironmentalScenario scenario;
    scenario.name = "Severe North Atlantic Storm";
    scenario.description = "Survival conditions - limited operations";
    scenario.wind.speed_knots = 45.0;
    scenario.wind.direction_deg = 90.0;
    scenario.sea_state = 7;
    scenario.visibility_km = 2.0;
    scenario.precipitation_mm_h = 30.0;
    scenario.water_temp_C = 8.0;
    scenario.salinity_ppt = 35.0;
    scenario.water_depth_m = 3000.0;
    scenario.should_be_operable = false;
    scenario.min_operational_readiness = 0.0;

    WindModel wind(scenario.wind);

    WaveParameters wave_params;
    wave_params.significant_wave_height_m =
        SeaState::douglasToWaveHeight(DouglasSeaState::HIGH);
    wave_params.peak_period_s = 12.0;
    WaveModel waves(wave_params);

    SeakeepingModel seakeeping;
    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        waves.getSignificantWaveHeight(),
        waves.getPeakPeriod(),
        90.0,
        8.0);

    OperationalEffectsModel ops_effects;
    EnvironmentalConditions conditions;
    conditions.wind_speed_m_s = wind.getWindSpeed();
    conditions.sea_state = scenario.sea_state;
    conditions.visibility_km = scenario.visibility_km;
    conditions.precipitation_mm_h = scenario.precipitation_mm_h;
    conditions.water_temperature_C = scenario.water_temp_C;
    conditions.salinity_ppt = scenario.salinity_ppt;
    conditions.ship_speed_m_s = 8.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // Severe conditions - major degradation expected
    EXPECT_LT(limits.overall_operational_readiness, 0.5);
    EXPECT_FALSE(limits.aircraft_ops.helicopter_launch);
    EXPECT_FALSE(limits.aircraft_ops.helicopter_recovery);
    EXPECT_FALSE(limits.speed_limits.full_speed_available);

    // Multiple limiting factors
    EXPECT_GT(limits.aircraft_ops.limitation_reasons.size(), 0);
    EXPECT_GT(limits.speed_limits.limitation_reasons.size(), 0);
}

TEST(EnvironmentalScenarios, TropicalOperations)
{
    // Scenario: Tropical operations
    EnvironmentalScenario scenario;
    scenario.name = "Tropical Operations";
    scenario.description = "Warm water, good visibility, light winds";
    scenario.wind.speed_knots = 10.0;
    scenario.wind.direction_deg = 180.0;
    scenario.sea_state = 2;
    scenario.visibility_km = 25.0;
    scenario.precipitation_mm_h = 0.0;
    scenario.water_temp_C = 28.0;
    scenario.salinity_ppt = 35.0;
    scenario.water_depth_m = 500.0;
    scenario.should_be_operable = true;
    scenario.min_operational_readiness = 0.95;

    WindModel wind(scenario.wind);

    WaveParameters wave_params;
    wave_params.significant_wave_height_m =
        SeaState::douglasToWaveHeight(DouglasSeaState::SMOOTH);
    wave_params.peak_period_s = 6.0;
    WaveModel waves(wave_params);

    SeakeepingModel seakeeping;
    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        waves.getSignificantWaveHeight(),
        waves.getPeakPeriod(),
        180.0,
        15.0);

    OperationalEffectsModel ops_effects;
    EnvironmentalConditions conditions;
    conditions.wind_speed_m_s = wind.getWindSpeed();
    conditions.sea_state = scenario.sea_state;
    conditions.visibility_km = scenario.visibility_km;
    conditions.precipitation_mm_h = scenario.precipitation_mm_h;
    conditions.water_temperature_C = scenario.water_temp_C;
    conditions.salinity_ppt = scenario.salinity_ppt;
    conditions.ship_speed_m_s = 15.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // Excellent conditions expected
    EXPECT_GT(
        limits.overall_operational_readiness,
        scenario.min_operational_readiness);
    EXPECT_GT(limits.optronics_performance.detection_range_factor, 0.95);
}

TEST(EnvironmentalScenarios, ArcticConditions)
{
    // Scenario: Arctic operations
    EnvironmentalScenario scenario;
    scenario.name = "Arctic Operations";
    scenario.description = "Cold water, reduced visibility, moderate seas";
    scenario.wind.speed_knots = 18.0;
    scenario.wind.direction_deg = 270.0;
    scenario.sea_state = 3;
    scenario.visibility_km = 5.0;
    scenario.precipitation_mm_h = 1.0;  // Light snow
    scenario.water_temp_C = -1.0;
    scenario.salinity_ppt = 32.0;  // Lower salinity due to ice melt
    scenario.water_depth_m = 200.0;
    scenario.should_be_operable = true;
    scenario.min_operational_readiness = 0.7;

    WindModel wind(scenario.wind);

    WaveParameters wave_params;
    wave_params.significant_wave_height_m =
        SeaState::douglasToWaveHeight(DouglasSeaState::SLIGHT);
    wave_params.peak_period_s = 6.5;
    WaveModel waves(wave_params);

    SeakeepingModel seakeeping;
    ShipMotions motions = seakeeping.computeMotions(
        "frigate",
        waves.getSignificantWaveHeight(),
        waves.getPeakPeriod(),
        270.0,
        10.0);

    OperationalEffectsModel ops_effects;
    EnvironmentalConditions conditions;
    conditions.wind_speed_m_s = wind.getWindSpeed();
    conditions.sea_state = scenario.sea_state;
    conditions.visibility_km = scenario.visibility_km;
    conditions.precipitation_mm_h = scenario.precipitation_mm_h;
    conditions.water_temperature_C = scenario.water_temp_C;
    conditions.salinity_ppt = scenario.salinity_ppt;
    conditions.ship_speed_m_s = 10.0;

    OperationalLimitations limits =
        ops_effects.computeEffects(conditions, motions);

    // Arctic conditions - some degradation expected
    EXPECT_GT(
        limits.overall_operational_readiness,
        scenario.min_operational_readiness);

    // Reduced visibility affects optronics
    EXPECT_LT(limits.optronics_performance.detection_range_factor, 0.9);
}

TEST(EnvironmentalScenarios, MultiDomainDetection)
{
    // Test scenario with simultaneous radar, sonar, and optical detection
    WindParameters wind_params;
    wind_params.speed_knots = 12.0;
    WindModel wind(wind_params);

    WaveParameters wave_params;
    wave_params.significant_wave_height_m = 1.5;
    wave_params.peak_period_s = 6.0;
    WaveModel waves(wave_params);

    // Setup environmental models
    RadarPropagationModel radar;
    AcousticPropagationModel acoustic;
    OpticalPropagationModel optical;

    // Test radar detection of surface target
    Eigen::Vector3d radar_tx(0, 0, 20);      // Ship radar at 20m height
    Eigen::Vector3d radar_tgt(10000, 0, 0);  // Target at 10km
    auto paths = radar.computePaths(radar_tx, radar_tgt, 10.0e9);
    EXPECT_GT(paths.size(), 0);  // Should have at least one path

    // Test acoustic detection of submarine
    Eigen::Vector3d sonar_tx(0, 0, -50);       // Sonar at 50m depth
    Eigen::Vector3d sonar_tgt(5000, 0, -100);  // Submarine at 100m depth
    double acoustic_loss =
        acoustic.computeTransmissionLoss(sonar_tx, sonar_tgt, 5000.0);
    EXPECT_LT(acoustic_loss, 100.0);  // TL should be reasonable at 5km

    // Test optical detection through atmosphere
    double optical_trans =
        optical.computeTransmittance(1000.0, 0.0, 550.0, 0.0);
    EXPECT_GT(optical_trans, 0.5);  // Should have good visibility at 1km
}

// ============================================================================
// Propagation Integration Tests
// ============================================================================

TEST(PropagationIntegration, UnderwaterAcousticDetection)
{
    // Test realistic underwater acoustic detection scenario
    AcousticPropagationModel acoustic;

    // Setup thermal profile (affects sound speed)
    ThermalProfileModel thermal_model;
    ThermalProfile profile = ThermalProfileModel::createDefaultProfile();
    acoustic.setThermalProfile(&profile);

    // Active sonar detection
    Eigen::Vector3d tx(0, 0, -100);         // Sonar at 100m depth
    Eigen::Vector3d target(8000, 0, -200);  // Target at 200m depth, 8km away

    double TL = acoustic.computeTransmissionLoss(tx, target, 5000.0);

    // Verify realistic transmission loss
    EXPECT_GT(TL, 60.0);   // Should have significant loss at 8km
    EXPECT_LT(TL, 100.0);  // But not excessive
}

TEST(PropagationIntegration, CrossMediumDetection)
{
    // Test air-to-water optical propagation
    OpticalPropagationModel optical;

    UnderwaterOpticalProperties water_props =
        UnderwaterOpticalModel::createWaterType("coastal");
    optical.setUnderwaterProperties(water_props);

    // Aircraft looking down into water
    double air_dist = 1000.0;   // 1km altitude
    double water_dist = 50.0;   // Looking 50m into water
    double wavelength = 550.0;  // Green light (nm)
    double angle = 30.0;        // 30° from vertical

    double trans =
        optical.computeTransmittance(air_dist, water_dist, wavelength, angle);

    // Should have significant attenuation
    EXPECT_LT(trans, 0.5);
    EXPECT_GT(trans, 0.0);
}

// ============================================================================
// Mission Scenario Tests
// ============================================================================

TEST(MissionScenario, NavalPatrolMission)
{
    // 24-hour patrol mission with varying conditions

    struct MissionPhase {
        double time_hours;
        std::string name;
        int sea_state;
        double wind_speed_knots;
    };

    std::vector<MissionPhase> phases = {
        {0.0, "Dawn - Departure", 2, 10.0},
        {6.0, "Morning - Transit", 3, 12.0},
        {12.0, "Noon - Patrol Area", 3, 15.0},
        {18.0, "Evening - Return", 2, 12.0},
        {24.0, "Night - Port Approach", 2, 8.0}};

    SeakeepingModel seakeeping;
    OperationalEffectsModel ops_effects;

    for (const auto& phase : phases) {
        WindParameters wind_params;
        wind_params.speed_knots = phase.wind_speed_knots;
        WindModel wind(wind_params);

        WaveParameters wave_params;
        wave_params.significant_wave_height_m = SeaState::douglasToWaveHeight(
            static_cast<DouglasSeaState>(phase.sea_state));
        wave_params.peak_period_s = 7.0;
        WaveModel waves(wave_params);

        ShipMotions motions = seakeeping.computeMotions(
            "frigate",
            waves.getSignificantWaveHeight(),
            waves.getPeakPeriod(),
            0.0,
            12.0);

        EnvironmentalConditions conditions;
        conditions.wind_speed_m_s = wind.getWindSpeed();
        conditions.sea_state = phase.sea_state;
        conditions.ship_speed_m_s = 12.0;

        OperationalLimitations limits =
            ops_effects.computeEffects(conditions, motions);

        // All phases should maintain operational capability
        EXPECT_GT(limits.overall_operational_readiness, 0.6)
            << "Phase: " << phase.name;
    }
}
