/**
 * @file operational_effects.cpp
 * @brief Operational effects implementation
 *
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_environment/vehicle/operational_effects.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim {
namespace environment {

// ============================================================================
// OperationalEffectsModel Implementation
// ============================================================================

OperationalEffectsModel::OperationalEffectsModel() {}

OperationalLimitations OperationalEffectsModel::computeEffects(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    OperationalLimitations limits;

    // Compute sensor performance
    limits.radar_performance = computeRadarPerformance(conditions, motions);
    limits.sonar_performance = computeSonarPerformance(conditions, motions);
    limits.optronics_performance =
        computeOptronicsPerformance(conditions, motions);
    limits.esm_performance = computeESMPerformance(conditions, motions);

    // Compute weapon performance
    limits.missile_performance = computeMissilePerformance(conditions, motions);
    limits.gun_performance = computeGunPerformance(conditions, motions);
    limits.torpedo_performance = computeTorpedoPerformance(conditions, motions);

    // Compute aircraft operations
    limits.aircraft_ops = computeAircraftOperations(conditions, motions);

    // Compute speed limitations
    limits.speed_limits =
        computeSpeedLimitations(conditions, motions, conditions.sea_state);

    // Overall readiness
    limits.overall_operational_readiness = computeOverallReadiness(limits);

    return limits;
}

SensorPerformance OperationalEffectsModel::computeRadarPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    SensorPerformance perf;

    // Base performance
    perf.detection_range_factor = 1.0;
    perf.false_alarm_rate_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.availability = 1.0;

    // Sea clutter impact
    double clutter_factor = getSeaClutterFactor(conditions.sea_state);
    perf.detection_range_factor *= clutter_factor;
    perf.false_alarm_rate_factor *= (2.0 - clutter_factor);  // Increase FAR
    if (clutter_factor < 0.9) {
        perf.degradation_reasons.push_back(
            "Sea clutter (Sea State " + std::to_string(conditions.sea_state) +
            ")");
    }

    // Rain attenuation (typical X-band radar at 9.5 GHz)
    double rain_factor =
        getRainAttenuationFactor(conditions.precipitation_mm_h, 9.5);
    perf.detection_range_factor *= rain_factor;
    if (rain_factor < 0.9) {
        perf.degradation_reasons.push_back(
            "Rain attenuation (" +
            std::to_string(conditions.precipitation_mm_h) + " mm/h)");
    }

    // Ship motion impact on antenna stabilization
    double motion_factor = getMotionStabilizationFactor(motions);
    perf.accuracy_factor *= motion_factor;
    if (motion_factor < 0.9) {
        perf.degradation_reasons.push_back(
            "Ship motion affecting stabilization");
    }

    // Severe conditions - system may be unavailable
    if (conditions.sea_state >= 7 || conditions.precipitation_mm_h > 50.0) {
        perf.availability = 0.8;
        perf.degradation_reasons.push_back("Severe weather conditions");
    }

    return perf;
}

SensorPerformance OperationalEffectsModel::computeSonarPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    SensorPerformance perf;

    perf.detection_range_factor = 1.0;
    perf.false_alarm_rate_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.availability = 1.0;

    // Sea noise impact (typical active sonar at 5 kHz)
    double noise_factor = getSeaNoiseFactor(conditions.sea_state, 5.0);
    perf.detection_range_factor *= noise_factor;
    perf.false_alarm_rate_factor *= (1.5 - 0.5 * noise_factor);
    if (noise_factor < 0.9) {
        perf.degradation_reasons.push_back(
            "Sea noise (Sea State " + std::to_string(conditions.sea_state) +
            ")");
    }

    // Self-noise from ship speed
    if (conditions.ship_speed_m_s > 10.0) {
        double self_noise_factor =
            1.0 - 0.05 * (conditions.ship_speed_m_s - 10.0);
        self_noise_factor = std::max(0.5, self_noise_factor);
        perf.detection_range_factor *= self_noise_factor;
        perf.degradation_reasons.push_back(
            "Self-noise from ship speed (" +
            std::to_string(conditions.ship_speed_m_s) + " m/s)");
    }

    // Hull-mounted sonar affected by ship pitch/roll
    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;
    if (roll_deg > 5.0) {
        perf.accuracy_factor *= (1.0 - 0.02 * (roll_deg - 5.0));
        perf.accuracy_factor = std::max(0.6, perf.accuracy_factor);
        perf.degradation_reasons.push_back("Ship roll affecting beam pattern");
    }

    // Severe conditions - towed array may need to be recovered
    if (conditions.sea_state >= 6) {
        perf.availability = 0.7;
        perf.degradation_reasons.push_back(
            "Towed array limitations in high sea state");
    }

    return perf;
}

SensorPerformance OperationalEffectsModel::computeOptronicsPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    SensorPerformance perf;

    perf.detection_range_factor = 1.0;
    perf.false_alarm_rate_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.availability = 1.0;

    // Visibility impact
    double vis_factor = getVisibilityFactor(conditions.visibility_km);
    perf.detection_range_factor *= vis_factor;
    if (vis_factor < 0.9) {
        perf.degradation_reasons.push_back(
            "Reduced visibility (" + std::to_string(conditions.visibility_km) +
            " km)");
    }

    // Rain/precipitation degrades optical
    if (conditions.precipitation_mm_h > 1.0) {
        double precip_factor =
            1.0 - 0.1 * std::log10(conditions.precipitation_mm_h + 1.0);
        precip_factor = std::max(0.3, precip_factor);
        perf.detection_range_factor *= precip_factor;
        perf.degradation_reasons.push_back("Precipitation degradation");
    }

    // Ship motion affects stabilization
    double motion_factor = getMotionStabilizationFactor(motions);
    perf.accuracy_factor *= motion_factor;
    if (motion_factor < 0.95) {
        perf.degradation_reasons.push_back("Motion affecting tracking");
    }

    // Heavy rain or fog - severely degraded
    if (conditions.precipitation_mm_h > 20.0 ||
        conditions.visibility_km < 1.0) {
        perf.availability = 0.5;
        perf.degradation_reasons.push_back("Severe visibility degradation");
    }

    return perf;
}

SensorPerformance OperationalEffectsModel::computeESMPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    (void)conditions;  // ESM less affected by weather

    SensorPerformance perf;

    perf.detection_range_factor = 1.0;
    perf.false_alarm_rate_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.availability = 1.0;

    // ESM mainly affected by antenna pointing accuracy
    double motion_factor = getMotionStabilizationFactor(motions);
    perf.accuracy_factor *= motion_factor;
    if (motion_factor < 0.95) {
        perf.degradation_reasons.push_back(
            "DF accuracy reduced by ship motion");
    }

    return perf;
}

WeaponPerformance OperationalEffectsModel::computeMissilePerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    WeaponPerformance perf;

    perf.can_launch = true;
    perf.max_range_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.probability_kill_factor = 1.0;

    // Ship motion affects launcher pointing
    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;
    double pitch_deg = std::abs(motions.pitch) * 180.0 / M_PI;

    // Launch envelope limits
    if (roll_deg > 15.0 || pitch_deg > 8.0) {
        perf.can_launch = false;
        perf.limitation_reasons.push_back(
            "Excessive ship motion (roll=" + std::to_string(roll_deg) +
            "°, pitch=" + std::to_string(pitch_deg) + "°)");
    }

    // Wind speed affects small missiles
    if (conditions.wind_speed_m_s > 20.0) {
        perf.accuracy_factor *= 1.2;  // Worse accuracy (higher CEP)
        perf.limitation_reasons.push_back("High wind speed");
    }

    // Sea state affects anti-ship missiles (sea-skimming)
    if (conditions.sea_state >= 5) {
        perf.max_range_factor *= 0.9;
        perf.probability_kill_factor *= 0.95;
        perf.limitation_reasons.push_back(
            "High sea state affecting sea-skimming");
    }

    // Severe conditions
    if (conditions.sea_state >= 7) {
        perf.can_launch = false;
        perf.limitation_reasons.push_back("Sea state too high for safe launch");
    }

    return perf;
}

WeaponPerformance OperationalEffectsModel::computeGunPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    (void)conditions;  // Gun mainly affected by ship motion

    WeaponPerformance perf;

    perf.can_launch = true;
    perf.max_range_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.probability_kill_factor = 1.0;

    // Ship motion affects gun stabilization
    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;
    double pitch_deg = std::abs(motions.pitch) * 180.0 / M_PI;

    // Accuracy degradation from motion
    if (roll_deg > 5.0) {
        perf.accuracy_factor *= (1.0 + 0.05 * (roll_deg - 5.0));
        perf.limitation_reasons.push_back("Roll affecting gun accuracy");
    }

    if (pitch_deg > 3.0) {
        perf.accuracy_factor *= (1.0 + 0.05 * (pitch_deg - 3.0));
        perf.limitation_reasons.push_back("Pitch affecting gun accuracy");
    }

    // Extreme motion - cannot fire
    if (roll_deg > 20.0 || pitch_deg > 12.0) {
        perf.can_launch = false;
        perf.limitation_reasons.push_back("Excessive motion prevents firing");
    }

    return perf;
}

WeaponPerformance OperationalEffectsModel::computeTorpedoPerformance(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    WeaponPerformance perf;

    perf.can_launch = true;
    perf.max_range_factor = 1.0;
    perf.accuracy_factor = 1.0;
    perf.probability_kill_factor = 1.0;

    // Ship motion affects torpedo tube pointing
    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;

    if (roll_deg > 10.0) {
        perf.can_launch = false;
        perf.limitation_reasons.push_back(
            "Excessive roll prevents torpedo launch");
    }

    // Sea noise affects torpedo homing
    double noise_factor = getSeaNoiseFactor(conditions.sea_state, 10.0);
    perf.probability_kill_factor *= noise_factor;
    if (noise_factor < 0.9) {
        perf.limitation_reasons.push_back("Sea noise affecting homing");
    }

    return perf;
}

AircraftOperations OperationalEffectsModel::computeAircraftOperations(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions) const
{
    AircraftOperations ops;

    ops.helicopter_launch = true;
    ops.helicopter_recovery = true;
    ops.fixed_wing_launch = true;
    ops.fixed_wing_recovery = true;

    // Compute flight deck motion (assume at stern, 50m from center)
    Eigen::Vector3d flight_deck_position(-50.0, 0.0, 10.0);  // Aft, elevated

    // Simplified vertical motion at flight deck
    // Heave + pitch-induced vertical motion
    double pitch_contribution = 50.0 * std::tan(motions.pitch);
    ops.flight_deck_motion_m =
        std::abs(motions.heave) + std::abs(pitch_contribution);

    // Helicopter launch limits
    if (ops.flight_deck_motion_m > 2.0) {
        ops.helicopter_launch = false;
        ops.limitation_reasons.push_back(
            "Flight deck motion too high for launch (" +
            std::to_string(ops.flight_deck_motion_m) + " m)");
    }

    // Helicopter recovery is more restrictive
    if (ops.flight_deck_motion_m > 1.5) {
        ops.helicopter_recovery = false;
        ops.limitation_reasons.push_back(
            "Flight deck motion too high for recovery");
    }

    // Wind speed limits
    if (conditions.wind_speed_m_s > 25.0) {
        ops.helicopter_launch = false;
        ops.helicopter_recovery = false;
        ops.limitation_reasons.push_back(
            "Wind speed too high (" +
            std::to_string(conditions.wind_speed_m_s) + " m/s)");
    }

    // Sea state limits (empirical)
    if (conditions.sea_state >= 6) {
        ops.helicopter_launch = false;
        ops.helicopter_recovery = false;
        ops.limitation_reasons.push_back(
            "Sea state too high (SS " + std::to_string(conditions.sea_state) +
            ")");
    }

    // Fixed-wing (carrier operations) - more restrictive
    if (ops.flight_deck_motion_m > 1.0 || conditions.sea_state >= 5) {
        ops.fixed_wing_launch = false;
        ops.fixed_wing_recovery = false;
        ops.limitation_reasons.push_back(
            "Conditions exceed carrier ops limits");
    }

    return ops;
}

SpeedLimitations OperationalEffectsModel::computeSpeedLimitations(
    const EnvironmentalConditions& conditions,
    const ShipMotions& motions,
    int sea_state) const
{
    (void)conditions;  // Speed mainly limited by sea state and motions

    SpeedLimitations limits;

    limits.max_safe_speed_m_s = 100.0;  // Very high default
    limits.recommended_speed_m_s = 15.0;
    limits.full_speed_available = true;

    // Sea state limitations (empirical)
    if (sea_state <= 3) {
        limits.max_safe_speed_m_s = 30.0;  // ~60 knots
        limits.recommended_speed_m_s = 20.0;
    } else if (sea_state == 4) {
        limits.max_safe_speed_m_s = 25.0;
        limits.recommended_speed_m_s = 15.0;
    } else if (sea_state == 5) {
        limits.max_safe_speed_m_s = 20.0;
        limits.recommended_speed_m_s = 12.0;
        limits.limitation_reasons.push_back("Speed limited by sea state 5");
    } else if (sea_state == 6) {
        limits.max_safe_speed_m_s = 15.0;
        limits.recommended_speed_m_s = 8.0;
        limits.full_speed_available = false;
        limits.limitation_reasons.push_back("Speed limited by sea state 6");
    } else if (sea_state >= 7) {
        limits.max_safe_speed_m_s = 10.0;
        limits.recommended_speed_m_s = 5.0;
        limits.full_speed_available = false;
        limits.limitation_reasons.push_back(
            "Speed severely limited by sea state " + std::to_string(sea_state));
    }

    // Acceleration limits - high accelerations indicate need to reduce speed
    double vert_accel_g = std::abs(motions.heave_acceleration) / 9.81;
    if (vert_accel_g > 0.3) {
        limits.max_safe_speed_m_s = std::min(limits.max_safe_speed_m_s, 12.0);
        limits.limitation_reasons.push_back("Excessive vertical accelerations");
    }

    // Slamming (bow impact) - heave and pitch together
    double pitch_deg = std::abs(motions.pitch) * 180.0 / M_PI;
    if (pitch_deg > 5.0 && std::abs(motions.heave) > 2.0) {
        limits.max_safe_speed_m_s = std::min(limits.max_safe_speed_m_s, 10.0);
        limits.limitation_reasons.push_back("Risk of hull slamming");
    }

    return limits;
}

double OperationalEffectsModel::computeOverallReadiness(
    const OperationalLimitations& limitations) const
{
    // Weighted average of system availability and performance

    double score = 0.0;
    double weight_sum = 0.0;

    // Sensors (30% weight total)
    score += limitations.radar_performance.detection_range_factor *
             limitations.radar_performance.availability * 0.10;
    score += limitations.sonar_performance.detection_range_factor *
             limitations.sonar_performance.availability * 0.10;
    score += limitations.optronics_performance.detection_range_factor *
             limitations.optronics_performance.availability * 0.05;
    score += limitations.esm_performance.availability * 0.05;
    weight_sum += 0.30;

    // Weapons (40% weight total)
    double missile_score =
        limitations.missile_performance.can_launch
            ? limitations.missile_performance.probability_kill_factor
            : 0.0;
    score += missile_score * 0.20;

    double gun_score = limitations.gun_performance.can_launch
                           ? (1.0 / limitations.gun_performance.accuracy_factor)
                           : 0.0;
    score += gun_score * 0.10;

    double torpedo_score =
        limitations.torpedo_performance.can_launch
            ? limitations.torpedo_performance.probability_kill_factor
            : 0.0;
    score += torpedo_score * 0.10;
    weight_sum += 0.40;

    // Aircraft operations (15% weight)
    double aircraft_score = 0.0;
    if (limitations.aircraft_ops.helicopter_launch &&
        limitations.aircraft_ops.helicopter_recovery) {
        aircraft_score = 1.0;
    } else if (
        limitations.aircraft_ops.helicopter_launch ||
        limitations.aircraft_ops.helicopter_recovery) {
        aircraft_score = 0.5;
    }
    score += aircraft_score * 0.15;
    weight_sum += 0.15;

    // Speed (15% weight)
    double speed_score =
        limitations.speed_limits.full_speed_available ? 1.0 : 0.5;
    score += speed_score * 0.15;
    weight_sum += 0.15;

    // Normalize
    double overall = score / weight_sum;

    return std::max(0.0, std::min(1.0, overall));
}

double OperationalEffectsModel::getSeaClutterFactor(int sea_state) const
{
    // Sea clutter reduces radar detection range
    // Based on empirical data (GIT model simplified)

    if (sea_state <= 1) {
        return 1.0;
    } else if (sea_state == 2) {
        return 0.95;
    } else if (sea_state == 3) {
        return 0.90;
    } else if (sea_state == 4) {
        return 0.85;
    } else if (sea_state == 5) {
        return 0.75;
    } else if (sea_state == 6) {
        return 0.65;
    } else {
        return 0.50;  // SS 7+
    }
}

double OperationalEffectsModel::getRainAttenuationFactor(
    double rain_rate_mm_h,
    double frequency_ghz) const
{
    // ITU-R P.838 simplified
    // Attenuation in dB/km

    if (rain_rate_mm_h < 0.1) {
        return 1.0;
    }

    // Simplified coefficients for frequency dependence
    double k = 0.0001 * std::pow(frequency_ghz, 2.0);
    double alpha = 1.0;

    // Specific attenuation (dB/km)
    double gamma = k * std::pow(rain_rate_mm_h, alpha);

    // Assume 10 km range, convert to factor
    double attenuation_dB = gamma * 10.0;
    double factor = std::pow(10.0, -attenuation_dB / 20.0);

    return std::max(0.3, factor);
}

double OperationalEffectsModel::getSeaNoiseFactor(
    int sea_state,
    double frequency_khz) const
{
    (void)frequency_khz;  // Simplified, could add frequency dependence

    // Sea noise increases with sea state
    // Wenz curves simplified

    if (sea_state <= 2) {
        return 1.0;
    } else if (sea_state == 3) {
        return 0.95;
    } else if (sea_state == 4) {
        return 0.90;
    } else if (sea_state == 5) {
        return 0.80;
    } else if (sea_state == 6) {
        return 0.70;
    } else {
        return 0.60;  // SS 7+
    }
}

double OperationalEffectsModel::getVisibilityFactor(double visibility_km) const
{
    // Optical range reduced by visibility

    if (visibility_km >= 10.0) {
        return 1.0;
    } else if (visibility_km >= 5.0) {
        return 0.8;
    } else if (visibility_km >= 2.0) {
        return 0.5;
    } else if (visibility_km >= 1.0) {
        return 0.3;
    } else {
        return 0.1;  // Heavy fog
    }
}

double OperationalEffectsModel::getMotionStabilizationFactor(
    const ShipMotions& motions) const
{
    // Ship motion affects stabilized sensors/weapons

    double roll_deg = std::abs(motions.roll) * 180.0 / M_PI;
    double pitch_deg = std::abs(motions.pitch) * 180.0 / M_PI;

    // Stabilization can compensate up to certain limits
    double factor = 1.0;

    if (roll_deg > 5.0) {
        factor *= (1.0 - 0.02 * (roll_deg - 5.0));
    }

    if (pitch_deg > 3.0) {
        factor *= (1.0 - 0.02 * (pitch_deg - 3.0));
    }

    return std::max(0.5, factor);
}

}  // namespace environment
}  // namespace lotusim
