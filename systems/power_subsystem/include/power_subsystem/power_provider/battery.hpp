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
 * @brief Abstract base class for batteries
 *
 * Inherits from PowerProvider and adds battery-specific behaviour:
 *   - charge from generator surplus
 *   - PowerLevel computation from voltage vs voltage_min thresholds
 *   - SOC tracking (initial value only for now —> FMU does not output SOC)
 *
 * SDF parameters read by Battery:
 *   capacity_ah  : battery capacity in Amp-hours
 *   initial_soc  : starting state of charge (0.0–1.0)
 *   voltage_min  : voltage below which battery is considered depleted (Volts)
 *
 * PowerLevel thresholds (hardcoded):
 *   NORMAL   : SOC > 0.20
 *   WARN     : 0.10 < SOC <= 0.20
 *   CRITICAL : 0.0  < SOC <= 0.10
 *   DEPLETED : SOC <= 0.0
 */
class Battery : public PowerProvider {
public:
    virtual ~Battery();
    // ----------------------------------------------------------------
    // PowerProvider interface —> implemented here for all batteries
    // ----------------------------------------------------------------
    /**
     * @brief returns the current health level based on state of charge
     *   NORMAL   : SOC > 0.20
     *   WARN     : 0.10 < SOC <= 0.20
     *   CRITICAL : 0.001  < SOC <= 0.10
     *   DEPLETED : SOC <= 0.001
     */
    PowerLevel powerLevel() const override
    {
        const float soc = getStateOfCharge();
        if (soc <= 0.001f) {
            return PowerLevel::DEPLETED;
        }
        if (soc <= 0.10f) {
            return PowerLevel::CRITICAL;
        }
        if (soc <= 0.20f) {
            return PowerLevel::WARN;
        }
        return PowerLevel::NORMAL;
    }

    /**
     * @brief Returns true when soc is 0
     */
    bool isDepleted() const override
    {
        return getStateOfCharge() <= 0.0f;
    }

    /**
     * @brief returns initial_soc for now
     */
    float getStateOfCharge() const override
    {
        return m_initialSoc;
    }

    /**
     * @brief approximate remaining energy in Wh
     *         TODO - it should be voltage() * current_capacity_A ------------
     */
    float availablePowerW() const override
    {
        return voltage() * m_capacityAh;
    }

    /**
     * @brief update current voltage
     */
    virtual void updateVoltage() = 0;

    /**
     * @brief battery can receive charge from a generator surplus
     */
    bool canReceiveCharge() const override
    {
        return true;
    }

    // ----------------------------------------------------------------
    // Pure virtual
    // ----------------------------------------------------------------
    /**
     * @brief push total current draw
     * @param currentA  net current on this provider (Amperes); positive =
     * discharge, negative = charge
     * @param dt        elapsed simulation time since last tick (sec)
     */
    void receiveLoad(float currentA, float dt) override = 0;

    /**
     * @brief current output voltage
     * @returns the value last received from the FMU
     */
    float voltage() const override = 0;

protected:
    /**
     * @brief subclasses call this
     *
     * @param name        battery name from SDF
     * @param node        node from PowerManager
     * @param capacity_ah  battery capacity from SDF capacity_ah
     * @param initial_soc  initial state of charge from SDF initial_soc (0–1)
     * @param voltage_min  depletion voltage from SDF voltage_min (V)
     */
    Battery(
        std::string name,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node)
        : PowerProvider(std::move(name), std::move(node))
    {
        m_capacityAh = sdf->Get<float>("capacity_ah", 100.0f).first;
        m_initialSoc = sdf->Get<float>("initial_soc", 1.0f).first;
        m_voltageMin = sdf->Get<float>("voltage_min", 36.0f).first;
    }

    // battery capacity in Amp-hour from SDF
    float m_capacityAh{0.0f};

    // Initial state of charge (0 - 1.0) from SDF initial_soc
    //  just returning initial value for now -----------------
    float m_initialSoc{1.0f};

    // min operating voltage from voltage_min
    // PowerLevel thresholds are derived from this value
    float m_voltageMin{0.0f};
};
}  // namespace lotusim::gazebo