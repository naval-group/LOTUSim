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
     *        Used by PowerManager for load shedding decisions and
     *        generator vs battery split calculation
     *        battery:   voltage * soc * capacity_ah  (remaining Wh estimate)
     *        generator: rated_output_w * fuel_ratio
     */
    virtual float availablePowerW() const = 0;

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
     *
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


    
    struct BatteryState{
        // To read the battery topics
        float current{0.0f};  // <[A] positive = discharging
        float voltage{0.0f};  // <[V]
    };

    /**
    * @param node                shared ROS 2 node (must outlive this object)
    * @param component_name      identifier used in log messages
    * @param current_topic       topic name
    */
    explicit PowerProvider(
        rclcpp::Node::SharedPtr node,
        const std::string& component_name,
        const std::string& current_topic,   // like "/hectate/OUT_I" but let it be defined by the node
        const std::string& voltage_topic
    );

    

    // subclasses must implement this - compute and return the power consumption [W]
    virtual float power_consume() const = 0;

    // return the latest battery state 
    BatteryState state() const;

    // to enable or disable this component
    // subclasses must respect this flag
    // true -> component allowed to draw power
    void set_active(bool on);

    // return if this component is allowed to draw power
    bool is_active() const;

    const std::string & component_name() const{
        return component_name_;
    }


private:
    // subscription callback - updates the cached battery state
    // updates state_.current and state_.voltage
    void on_current(
        const std_msgs::msg::Float64::SharedPtr msg
    );
    void on_voltage(
        const std_msgs::msg::Float64::SharedPtr msg
    );

    std::string component_name_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr voltage_sub_;
    // cached battery readings updated in the subscription callback
    // using atomic so power_consume() can read without a mutex
    std::atomic<float> state_current_{0.0f};
    std::atomic<float> state_voltage_{0.0f};
    // if this component is allowed to consume power
    std::atomic<bool> active_{true};
};
} // namespace lotusim::gazebo