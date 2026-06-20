/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <gz/common/Console.hh>

#include "power_subsystem/power_provider/power_provider.hpp"

namespace lotusim::gazebo {

/**
 * @brief Abstract base class for generators
 *
 * Inherits from PowerProvider and adds generator behaviour:
 *   - fuel consumption tracking
 *   - surplus energy computation for battery charging
 *   - PowerLevel computation from fuel ratio thresholds
 *
 * A generator produces a stable nominal voltage as long as it has fuel
 * If the generator output exceeds the current demand, the surplus
 * is forwarded to PowerManager which calls receiveCharge() on the
 * active battery
 *
 * SDF parameters read by Generator:
 *   fuel_capacity    : full size tank in litres
 *   fuel_level_start : starting fuel level in litres
 *   rated_output_w   : maximum continuous power output (Watts)
 *   efficiency       : fuel-to-electrical conversion ratio (0.0–1.0)
 *   voltage_nominal  : stable output voltage (Volts)
 *
 * PowerLevel thresholds (from fuel ratio):
 *   NORMAL   : fuel > 25%
 *   WARN     : 10% < fuel <= 25%
 *   CRITICAL : 5%  < fuel <= 10%
 *   DEPLETED : fuel <= 5%
 */
class Generator : public PowerProvider {
public:
    // ----------------------------------------------------------------
    // PowerProvider interface -> implemented here for all generators
    // ----------------------------------------------------------------
    /**
     * @brief returns health level based on remaining fuel ratio
     *   NORMAL   : fuel > 25%
     *   WARN     : 10% < fuel <= 25%
     *   CRITICAL : 1%  < fuel <= 10%
     *   DEPLETED : fuel <= 1%
     */
    PowerLevel powerLevel() const override
    {
        const float ratio = fuelRatio();
        if (ratio <= 0.01f) {
            return PowerLevel::DEPLETED;
        }
        if (ratio <= 0.10f) {
            return PowerLevel::CRITICAL;
        }
        if (ratio <= 0.20f) {
            return PowerLevel::WARN;
        }
        return PowerLevel::NORMAL;
    }

    /**
     * @brief returns true when fuel has dropped to or below 5%
     */
    bool isDepleted() const override
    {
        return powerLevel() == PowerLevel::DEPLETED;
    }

    /**
     * @brief returns fuel ratio 0.0 - 1.0
     *        fuel_level_start / fuel_capacity
     */
    float getStateOfCharge() const override
    {
        return fuelRatio();
    }

    /**
     * @brief power the generator can sustain right now (W)
     */
    float availablePowerW() const override
    {
        return m_rated_output_W * fuelRatio();
    }

    /**
     * @brief generators don't accept charge from other sources
     */
    bool canReceiveCharge() const override
    {
        return false;
    }

    /**
     * @brief generators output a stable nominal voltage while fuelled
     */
    float voltage() const override
    {
        return isDepleted() ? 0.0f : m_voltage_nominal;
    }

    /**
     * @brief compute surplus charging current available
     *
     * called by PowerManager after receiveLoad() to determine how much
     * charge to push to the active battery
     *
     * @param busCurrentA  total current drawn by all active consumers (A)
     * returns surplus current available for charging (A), 0 if none
     */
    // AHOY merge this with receiving load. no point having this
    float surplusChargingCurrent(float busCurrentA) const
    {
        if (isDepleted()) {
            return 0.0f;
        }
        const float demandW = busCurrentA * m_voltage_nominal;
        const float surplusW = availablePowerW() - demandW;
        return surplusW > 0.0f ? (surplusW / m_voltage_nominal) : 0.0f;
    }

    float fuelRatio() const
    {
        if (m_fuel_capacity <= 0.0f) {
            return 0.0f;
        }
        return m_fuel_level / m_fuel_capacity;
    }

    // ----------------------------------------------------------------
    // Pure virtual -> subclasses must implement
    // ----------------------------------------------------------------

    /**
     * @brief consume fuel proportional to current load over dt
     *        updates m_fuelLevel
     *
     * @param currentA  net current on this provider (Amperes); positive =
     * discharge, negative = charge
     * @param dt        elapsed simulation time since last tick (s)
     */
    void receiveLoad(float currentA, float dt) override = 0;

protected:
    /**
     * @brief Subclasses call this constructor
     *
     * @param name           generator name from SDF
     * @param node           shared node from PowerManager
     * @param fuel_capacity  full tank size in litres
     * @param fuel_start     starting fuel
     * @param rated_output_w maximum continuous power output (W)
     * @param efficiency     fuel-to-electrical ratio (0.0–1.0)
     * @param voltage_nominal stable output voltage (V)
     */
    Generator(
        std::string name,
        const sdf::ElementPtr& _sdf,
        rclcpp::Node::SharedPtr node)
        : PowerProvider(std::move(name), std::move(node))
    {
        m_fuel_level = _sdf->Get<float>("fuel_level_start", 400.0f).first;
        m_fuel_capacity = _sdf->Get<float>("fuel_capacity", 500.0f).first;
        m_rated_output_W = _sdf->Get<float>("rated_output_w", 5000.0f).first;
        m_efficiency = _sdf->Get<float>("efficiency", 0.35f).first;
        m_voltage_nominal = _sdf->Get<float>("voltage_nominal", 48.0f).first;
        m_fuel_type = _sdf->Get<std::string>("fuel_type", "diesel").first;
    }

    // current fuel level
    float m_fuel_level{0.0f};

    // full tank capacity
    float m_fuel_capacity{0.0f};

    // maximum continuous power output from SDF rated_output_w (W)
    float m_rated_output_W{0.0f};

    // fuel-to-electrical conversion ratio from SDF efficiency (0.0–1.0)
    float m_efficiency{1.0f};

    // stable output voltage from SDF voltage_nominal (V)
    float m_voltage_nominal{0.0f};

    // fuel type -> different consumption
    std::string m_fuel_type;
};
}  // namespace lotusim::gazebo