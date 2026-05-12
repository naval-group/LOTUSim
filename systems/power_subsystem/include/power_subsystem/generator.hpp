#pragma once

#include "power_provider.hpp"
#include <gz/common/Console.hh>

namespace lotusim::gazebo{

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
class Generator : public PowerProvider{
public:
    // ----------------------------------------------------------------
    // PowerProvider interface -> implemented here for all generators
    // ----------------------------------------------------------------
    /**
     * @brief returns health level based on remaining fuel ratio
     *   NORMAL   : fuel > 25%
     *   WARN     : 10% < fuel <= 25%
     *   CRITICAL : 5%  < fuel <= 10%
     *   DEPLETED : fuel <= 5%
     */
    PowerLevel powerLevel() const override
    {
        const float ratio = fuelRatio();
        if (ratio <= 0.05f) { return PowerLevel::DEPLETED; }
        if (ratio <= 0.10f) { return PowerLevel::CRITICAL; }
        if (ratio <= 0.25f) { return PowerLevel::WARN; }
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
        return m_ratedOutputW * fuelRatio();
    }

    /**
     * @brief generators don't accept charge from other sources
     */
    bool canReceiveCharge() const override { return false; }

    /**
     * @brief generators output a stable nominal voltage while fuelled
     */
    float voltage() const override
    {
        return isDepleted() ? 0.0f : m_voltageNominal;
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
    float surplusChargingCurrent(float busCurrentA) const
    {
        if (isDepleted()) { return 0.0f; }
        const float demandW  = busCurrentA * m_voltageNominal;
        const float surplusW = m_ratedOutputW - demandW;
        return surplusW > 0.0f ? (surplusW / m_voltageNominal) : 0.0f;
    }

    // ----------------------------------------------------------------
    // Pure virtual -> subclasses must implement
    // ----------------------------------------------------------------

    /**
     * @brief consume fuel proportional to current load over dt
     *        updates m_fuelLevel
     *
     * @param currentA  total current drawn from this generator (A)
     * @param dt        elapsed simulation time since last tick (s)
     */
    void receiveLoad(float currentA, float dt) override =


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
        rclcpp::Node::SharedPtr node,
        float fuel_capacity,     
        float fuel_level_start, 
        float rated_output_w,
        float efficiency,
        float voltage_nominal)
        : PowerProvider(std::move(name), std::move(node))
        , m_fuelCapacity(fuel_capacity)
        , m_fuelLevel(fuel_level_start)
        , m_ratedOutputW(rated_output_w)
        , m_efficiency(efficiency)
        , m_voltageNominal(voltage_nominal)
    {}

    float fuelRatio() const
    {
        if (m_fuelCapacity <= 0.0f) { return 0.0f; }
        return m_fuelLevel / m_fuelCapacity;  
    }

    // current fuel level
    float m_fuelLevel{0.0f};

    // full tank capacity
    float m_fuelCapacity{0.0f};

    // maximum continuous power output from SDF rated_output_w (W)
    float m_ratedOutputW{0.0f};

    // fuel-to-electrical conversion ratio from SDF efficiency (0.0–1.0)
    float m_efficiency{1.0f};

    // stable output voltage from SDF voltage_nominal (V)
    float m_voltageNominal{0.0f};
};
} // namespace lotusim::gazebo