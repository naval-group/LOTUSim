/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <sdf/Element.hh>
#include "rclcpp/rclcpp.hpp"

namespace lotusim::gazebo{
/**
 * @brief Health level of a power provider
 *
 * Threshold levels (computed from sdf voltage_min) PowerManager behaviour per level:
 *   NORMAL   : no action
 *   WARN     : warning log + shed priority 4 consumers
 *   CRITICAL : warning log + shed priority 3 and below
 *   DEPLETED : info log + switch to next battery if available
 */
enum class PowerLevel
{
    NORMAL,
    WARN,
    CRITICAL,
    DEPLETED
};
/**
 * @brief Abstract base class for all power sources on a vessel.
 *
 * Defines the contract between any power source and PowerManager.
 * PowerManager only ever calls these methods, it has no knowledge
 * of whether the provider is a battery, a generator, or anything else.
 *
 * Two concrete abstract subclasses inherit from this:
 *   - Battery   : stores electrical energy, interfaces with FMU,
 *                 can receive charge from a generator surplus
 *   - Generator : converts fuel to electrical energy, cannot receive charge
 *
 * Concrete implementations (NuclearBattery, DieselGenerator, ...) inherit
 * from those subclasses and implement type-specific behaviour.
 *
 * receiveLoad() takes dt (seconds) because providers need to integrate
 * over time to compute physically correct SOC and fuel consumption:
 *   Battery   : soc -= (currentA * dt) / capacity_ah
 *   Generator : fuel -= (power * dt) / (efficiency * 3600)
 */

class PowerProvider {
public:
    virtual ~PowerProvider() = default;

    // ----------------------------------------------------------------
    // Pure virtual —> every subclass must implement these
    // ----------------------------------------------------------------

    /**
     * @brief Called by PowerManager with the current bus load
     *        Battery: publishes, integrates SOC over dt
     *        Generator: computes fuel consumed over dt
     * @param currentA  total current drawn from this provider (Amperes)
     * @param dt        elapsed simulation time since last tick (seconds)
     */
    virtual void receiveLoad(float currentA, float dt) = 0;

    /**
     * @brief Current output voltage of this provider (V)
     *        Battery:   last value received from FMU topic callback
     *        Generator: stable nominal voltage (simplified)
     */
    virtual float voltage() const = 0;

    /**
     * @brief normalised remaining capacity (0.0 to 1.0)
     *        Battery:   remaining charge ratio (SOC)
     *        Generator: remaining fuel ratio
     */
    virtual float getStateOfCharge() const = 0;

    /**
     * @brief approximate power the provider can sustain right now (W)
     *        Used by PowerManager for load shedding decisions
     *        battery:   voltage * soc * capacity_ah  (remaining Wh estimate)
     *        generator: rated_output_w * fuel_ratio
     */
    virtual float availablePowerW() const = 0;

    /**
     * @brief Returns the current health level of this provider
     *        computed internally from voltage vs voltage_min thresholds
     *        PowerManager uses this to decide shedding and switching
     */
    virtual PowerLevel powerLevel() const = 0;

    /**
     * @brief if this provider can no longer supply power
     *        battery:   soc <= 0.0
     *        generator: fuel_level <= 0.0
     */
    virtual bool isDepleted() const = 0;

    // ----------------------------------------------------------------
    // Virtual with defaults —> subclasses override only if relevant
    // ----------------------------------------------------------------

    /**
     * @brief if this provider can receive charge from a generator surplus
     *        Battery overrides → true
     *        Generator keeps default
     */
    virtual bool canReceiveCharge() const { return false; }

    /**
     * @brief push surplus energy into this provider
     *        Default: no-op. Generators ignore this 
     *        Battery implements: soc += (currentA * dt) / capacity_ah,
     *        clamped to 1.0
     * @param currentA  Surplus current available for charging (Amperes)
     * @param dt        Elapsed simulation time since last tick (seconds)
     */
    virtual void receiveCharge(float currentA, float dt) { (void)currentA; (void)dt; }

    // ----------------------------------------------------------------
    // Non-virtual —> same for all providers
    // ----------------------------------------------------------------
    /**
     * @brief name of this provider from SDF
     *        Used in log messages and topics
     */
    const std::string& name() const { return m_name; }

protected:
    /**
     * @brief only subclasses can instantiate
     * @param name  provider name from SDF
     * @param node  shared node owned by PowerManager
     */
    PowerProvider(std::string name, rclcpp::Node::SharedPtr node)
        : m_name(std::move(name))
        , m_node(std::move(node))
    {}
 
    // Provider name from SDF name attribute
    std::string m_name;
 
    // Shared node, borrowed from PowerManager
    rclcpp::Node::SharedPtr m_node;
};
} // namespace lotusim::gazebo