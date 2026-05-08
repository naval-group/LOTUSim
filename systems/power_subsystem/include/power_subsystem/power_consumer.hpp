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

 #include <string>
 #include <memory>
 #include <sdf/Element.hh>
 #include <rclcpp/rclcpp.hpp>
 #include <gz/sim/EntityComponentManager.hh>

 namespace lotusim::gazebo{
    /**
 * @brief Abstract base class for all power-consuming components on a vessel
 *
 * Defines the contract between any power consumer and PowerManager
 * Subclasses inherit from this and implement:
 *   - drawnCurrent() : their individual current draw at this tick
 *   - update()       : any per-tick internal state update
 *
 * Responsibility split:
 *   PowerConsumer  : individual component draw + on/off state
 *   PowerManager   : summation across all consumers, voltage distribution,
 *                    load shedding decisions
 *
 * Priority groups for consumers:
 *   1 = safety critical        — never shed by PowerManager
 *   2 = operationally important
 *   3 = mission systems        — default if omitted
 *   4 = non-essential          — shed first
 */
class PowerConsumer {
public:
    virtual ~PowerConsumer() = default;

    // ----------------------------------------------------------------
    // Pure virtual —> every subclass must implement these
    // ----------------------------------------------------------------

    /**
     * @brief Individual current draw of this component (Amp)
     *        Called by PowerManager to sum total bus load
     *        returns 0.0 if !isActive() or if m_voltage <= 0.0
     *        Formula: I = P / V
     *        SensorConsumer  : nominalPowerW() / m_voltage
     *        ThrusterConsumer: (nominalPowerW() * rpm_ratio²) / m_voltage
     */
    virtual float drawnCurrent() const = 0;

    /**
     * @brief called by PowerManager
     *        Subclass updates any internal state that changes
     *        SensorConsumer  : nothing (fixed draw, no internal state to update)
     *        ThrusterConsumer: reads current RPM,
     *                          updates m_rpmRatio
     */
    virtual void update(gz::sim::EntityComponentManager& _ecm) = 0;

    // ----------------------------------------------------------------
    // Virtual with defaults —> subclasses override only if relevant
    // ----------------------------------------------------------------

    /**
     * @brief Called by PowerManager when a new component is added at runtime
     *        Default: no-op
     *        override if the consumer needs to
     */
    virtual void eachNew() {}

    /**
     * @brief Called by PowerManager when a component is removed at runtime
     *        Default: no-op
     *        Override if the consumer needs to
     */
    virtual void eachDelete() {}

    /**
     * @brief Deactivates this consumer —> called by PowerManager during
     *        load shedding or full power loss
     *        Subclass should override to add type-specific shutdown behaviour
     *        (e.g. ThrusterConsumer sets rpm to zero, SensorConsumer
     *        publishes a power_state = OFF notification)
     */
    virtual void deactivate()
    {
        m_active = false;
    }

    /**
     * @brief Reactivates this consumer after it was deactivated
     *        Sets m_active = true so drawnCurrent() resumes 
     *        Called by PowerManager when power margin recovers
     *        TODO: PowerManager currently has no reactivation logic ->
     *        call reactivate() in shedLoadsIfNeeded() when
     *        PowerLevel returns to NORMAL
     */
    virtual void reactivate()
    {
        m_active = true;
    }
 

    // ----------------------------------------------------------------
    // Non-virtual —> same for all providers
    // ----------------------------------------------------------------
    
    /**
     * @brief receives the current bus voltage from PowerManager
     *        Called before drawnCurrent() so the consumer
     *        always uses an up-to-date voltage for its I calculation
     *        Guards against zero voltage
     */
    void receiveVoltage(float v)
    {
        m_voltage = (v > 0.0f) ? v : 0.0f;
    }
    
    // check if this consumer is active
    // False after deactivate() is called
    bool isActive() const { return m_active; }

    // priority group for load shedding (1–4)
    // read from SDF priority attribute. default = 3
    int priority() const { return m_priority; }
    
    // Nominal power draw in Watts —> read from SDF nominal_w
    // used by PowerManager::shedLoadsIfNeeded() for demand estimation
    float nominalPowerW() const { return m_nominalPowerW; }

    // used in log messages
    const std::string& name() const { return m_name; }

protected:
    /**
     * @brief only subclasses can instantiate
     *
     * @param name        component name from SDF
     * @param nominalW    nominal power draw in Watts from SDF nominal_w
     * @param priority    Load shedding priority from SDF priority (default 3)
     * @param node        shared node owned by PowerManager
     */
     PowerConsumer(
        std::string name,
        float nominalW,
        int priority,
        rclcpp::Node::SharedPtr node)
        : m_nominalPowerW(nominalW)   
        , m_node(std::move(node))   
        , m_name(std::move(name))  
        , m_priority(priority)
    {}

    // last voltage pushed by PowerManager via receiveVoltage()
    // used by subclasses in drawnCurrent()
    float m_voltage{0.0f};

    // nominal power draw in Watts, from SDF nominal_w
    float m_nominalPowerW{0.0f};
 
    // Shared node
    rclcpp::Node::SharedPtr m_node;

private:
        std::string m_name;
        int m_priority{3};
        bool m_active{true};
};
} //namespace lotusim::gazebo