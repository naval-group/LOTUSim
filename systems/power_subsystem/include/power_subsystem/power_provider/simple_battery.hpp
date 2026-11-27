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

#include "power_subsystem/power_provider/battery.hpp"

namespace lotusim::gazebo {

/**
 * @brief minimal battery for integration testing
 *
 * For now, testing: stubs out FMU interaction with coulomb counting and a
 * voltage linear in the state of charge:
 *
 *   remaining_ah -= currentA * dt / 3600     (dt in s; currentA < 0 charges)
 *   voltage = voltage_min + soc * (voltage_nominal - voltage_min)
 *
 * replace receiveLoad(), voltage() and receiveCharge() with
 * FMU calls once the FMU wrapper interface is available
 *
 * SDF format (a <lotusim_power> element of the vessel's model):
 *   <lotusim_power>
 *       <name>main_battery</name>
 *       <type>simple_battery</type>
 *       <capacity_ah>100</capacity_ah>
 *       <initial_soc>1.0</initial_soc>
 *       <voltage_min>36.0</voltage_min>
 *       <voltage_nominal>48.0</voltage_nominal>
 *   </lotusim_power>
 */
class SimpleBattery : public Battery {
public:
    /**
     * @param name   Battery name from SDF
     * @param _sdf   reads capacity_ah, initial_soc,
     *               voltage_min, voltage_nominal
     * @param node   node from PowerManager
     */
    SimpleBattery(
        const std::string& battery_name,
        const std::string& vessel_name,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<spdlog::logger> logger);

    ~SimpleBattery() = default;

    // ----------------------------------------------------------------
    // Battery interface
    // ----------------------------------------------------------------

    /**
     * @brief Integrates the charge over dt: currentA > 0 discharges,
     *        currentA < 0 charges, clamped to [0, capacity_ah]
     *        stub for FMU call — replace when wrapper is available
     * -----------------
     */
    void receiveLoad(float currentA, float dt) override;

    /**
     * @brief returns last computed voltage (V)
     *        real implementation reads from FMU output.
     */
    float voltage() const override;

    float getStateOfCharge() const override;

    void updateVoltage() override;

    float availablePowerW() const override;

private:
    // current battery voltage: starts at voltage_nominal, drains toward
    // voltage_min as load is applied. Updated by receiveLoad()
    float m_voltage{48.0f};

    // full-charge voltage from SDF voltage_nominal
    float m_voltage_nominal{48.0f};

    // tracks remaining charge. A double: a float near 100 Ah cannot take
    // the few micro amp-hours of a small current over one step.
    double m_remainingAh{100.0};
};

}  // namespace lotusim::gazebo