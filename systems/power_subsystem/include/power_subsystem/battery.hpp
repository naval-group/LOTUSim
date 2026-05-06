#pragma once

#include "power_provide.hpp"
#include <gz/common/Console.hh>

namespace lotusim::gazebo{
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
 *   NORMAL   : voltage > voltage_min * 1.15
 *   WARN     : voltage_min * 1.05 < voltage <= voltage_min * 1.15
 *   CRITICAL : voltage_min        < voltage <= voltage_min * 1.05
 *   DEPLETED : voltage <= voltage_min
 */
class Battery : public PowerProvider{
public:
    // ----------------------------------------------------------------
    // PowerProvider interface —> implemented here for all batteries
    // ----------------------------------------------------------------
    /**
     * @brief returns the current health level based on voltage output
     *        thresholds are computed from voltage_min in SDF
     *
     *   NORMAL   : voltage > voltage_min * 1.15   (>15% above min)
     *   WARN     : voltage_min * 1.05 < v <= voltage_min * 1.15
     *   CRITICAL : voltage_min < v <= voltage_min * 1.05  (<5% above min)
     *   DEPLETED : voltage <= voltage_min
     */
    PowerLevel powerLevel() const override
    {
        const float v = voltage();
        if (v <= m_voltageMin) {
            return PowerLevel::DEPLETED;
        }
        if (v <= m_voltageMin * 1.05f) {
            return PowerLevel::CRITICAL;
        }
        if (v <= m_voltageMin * 1.15f) {
            return PowerLevel::WARN;
        }
        return PowerLevel::NORMAL;
    }

    /**
     * @brief Returns true when voltage has dropped to or below voltage_min
     */
    bool isDepleted() const override
    {
        return powerLevel() == PowerLevel::DEPLETED;
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
     *         TODO - will need to update when generators are implemented ------------
     */
    float availablePowerW() const override
    {
        return voltage() * m_capacityAh;
    }

    /**
     * @brief battery can receive charge from a generator surplus
     */
    bool canReceiveCharge() const override { return true; }

    // ----------------------------------------------------------------
    // Pure virtual
    // ----------------------------------------------------------------
    /**
     * @brief push total current draw
     * @param currentA  total current drawn from this battery (Amp)
     * @param dt        elapsed simulation time since last tick (sec)
     */
    void receiveLoad(float currentA, float dt) override = 0;

    /**
     * @brief current output voltage 
     *        subclass returns the value last received from the FMU
     */
    float voltage() const override = 0;

    /**
     * @brief push surplus current into this battery for charging
     *
     * @param currentA  Surplus charging current (Amp)
     * @param dt        Elapsed simulation time since last tick (sec)
     */
    void receiveCharge(float currentA, float dt) override = 0;

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
        rclcpp::Node::SharedPtr node,
        float capacity_ah,
        float initial_soc,
        float voltage_min)
        : PowerProvider(std::move(name), std::move(node))
        , m_capacityAh(capacity_ah)
        , m_initialSoc(initial_soc)
        , m_voltageMin(voltage_min)
    {}

    // battery capacity in Amp-hour from SDF
    float m_capacityAh{0.0f};

    //Initial state of charge (0 - 1.0) from SDF initial_soc
    // just returning initial value for now -----------------
    float m_initialSoc{1.0f};

    // min operating voltage from voltage_min
    // PowerLevel thresholds are derived from this value
    float m_voltageMin{0.0f};
};
}   // namespace lotusim::gazebo