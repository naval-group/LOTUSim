/**
 * @file operational_effects.hpp
 * @brief Operational effects of environmental conditions on ship systems
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#ifndef LOTUSIM_ENVIRONMENT_VEHICLE_OPERATIONAL_EFFECTS_HPP_
#define LOTUSIM_ENVIRONMENT_VEHICLE_OPERATIONAL_EFFECTS_HPP_

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include "lotusim_environment/vehicle/seakeeping_model.hpp"

namespace lotusim {
namespace environment {

/**
 * @brief Environmental condition summary
 */
struct EnvironmentalConditions {
    // Meteorological
    double wind_speed_m_s{0.0};
    double wind_direction_deg{0.0};
    int sea_state{0};  // Douglas scale 0-9
    double significant_wave_height_m{0.0};
    double visibility_km{10.0};
    double precipitation_mm_h{0.0};

    // Oceanographic
    double water_temperature_C{15.0};
    double salinity_ppt{35.0};
    double current_speed_m_s{0.0};

    // Ship state
    double ship_speed_m_s{0.0};
    double ship_heading_deg{0.0};
};

/**
 * @brief Sensor performance degradation
 */
struct SensorPerformance {
    double detection_range_factor{1.0};   // Multiplier on nominal range (0-1)
    double false_alarm_rate_factor{1.0};  // Multiplier on nominal FAR (1+)
    double accuracy_factor{1.0};  // Multiplier on nominal accuracy (0-1)
    double availability{1.0};     // Probability system is operable (0-1)
    std::vector<std::string> degradation_reasons;
};

/**
 * @brief Weapon system performance degradation
 */
struct WeaponPerformance {
    bool can_launch{true};                // Can weapon be launched
    double max_range_factor{1.0};         // Multiplier on max range (0-1)
    double accuracy_factor{1.0};          // Multiplier on CEP (1+, worse)
    double probability_kill_factor{1.0};  // Multiplier on Pk (0-1)
    std::vector<std::string> limitation_reasons;
};

/**
 * @brief Aircraft operations limitations
 */
struct AircraftOperations {
    bool helicopter_launch{true};      // Can launch helicopter
    bool helicopter_recovery{true};    // Can recover helicopter
    bool fixed_wing_launch{true};      // Can launch fixed-wing (carrier)
    bool fixed_wing_recovery{true};    // Can recover fixed-wing (carrier)
    double flight_deck_motion_m{0.0};  // Vertical motion at flight deck
    std::vector<std::string> limitation_reasons;
};

/**
 * @brief Ship speed limitations
 */
struct SpeedLimitations {
    double max_safe_speed_m_s{100.0};    // Maximum safe speed
    double recommended_speed_m_s{15.0};  // Recommended cruise speed
    bool full_speed_available{true};
    std::vector<std::string> limitation_reasons;
};

/**
 * @brief Complete operational limitations summary
 */
struct OperationalLimitations {
    SensorPerformance radar_performance;
    SensorPerformance sonar_performance;
    SensorPerformance optronics_performance;
    SensorPerformance esm_performance;

    WeaponPerformance missile_performance;
    WeaponPerformance gun_performance;
    WeaponPerformance torpedo_performance;

    AircraftOperations aircraft_ops;
    SpeedLimitations speed_limits;

    double overall_operational_readiness{1.0};  // 0-1, composite score
};

/**
 * @brief Model operational effects of environment on ship systems
 */
class OperationalEffectsModel {
public:
    OperationalEffectsModel();

    /**
     * @brief Compute all operational effects
     *
     * @param conditions Environmental conditions
     * @param motions Ship motions from seakeeping
     * @return Complete operational limitations
     */
    OperationalLimitations computeEffects(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute radar performance degradation
     */
    SensorPerformance computeRadarPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute sonar performance degradation
     */
    SensorPerformance computeSonarPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute optical/IR sensor performance
     */
    SensorPerformance computeOptronicsPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute ESM performance
     */
    SensorPerformance computeESMPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute missile launch envelope limitations
     */
    WeaponPerformance computeMissilePerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute gun performance limitations
     */
    WeaponPerformance computeGunPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute torpedo performance
     */
    WeaponPerformance computeTorpedoPerformance(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute aircraft operations limitations
     */
    AircraftOperations computeAircraftOperations(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions) const;

    /**
     * @brief Compute speed limitations
     */
    SpeedLimitations computeSpeedLimitations(
        const EnvironmentalConditions& conditions,
        const ShipMotions& motions,
        int sea_state) const;

    /**
     * @brief Compute overall operational readiness (0-1)
     */
    double computeOverallReadiness(
        const OperationalLimitations& limitations) const;

private:
    /**
     * @brief Get sea clutter impact on radar
     */
    double getSeaClutterFactor(int sea_state) const;

    /**
     * @brief Get rain attenuation factor
     */
    double getRainAttenuationFactor(double rain_rate_mm_h, double frequency_ghz)
        const;

    /**
     * @brief Get sea noise impact on sonar
     */
    double getSeaNoiseFactor(int sea_state, double frequency_khz) const;

    /**
     * @brief Get visibility impact on optics
     */
    double getVisibilityFactor(double visibility_km) const;

    /**
     * @brief Get motion impact on sensor stabilization
     */
    double getMotionStabilizationFactor(const ShipMotions& motions) const;
};

}  // namespace environment
}  // namespace lotusim

#endif  // LOTUSIM_ENVIRONMENT_VEHICLE_OPERATIONAL_EFFECTS_HPP_
