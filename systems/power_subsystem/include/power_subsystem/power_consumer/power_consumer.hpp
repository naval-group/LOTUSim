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

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <string>
#include <string_view>

#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {
enum class ConsumerType
{
    Unknown,
    Sensor,
    Thruster
};

inline std::string_view toString(ConsumerType type)
{
    switch (type) {
        case ConsumerType::Unknown:
            return "unknown";
        case ConsumerType::Sensor:
            return "sensor";
        case ConsumerType::Thruster:
            return "thruster";
    }
    return "";
}

inline std::optional<ConsumerType> consumerTypeFromString(std::string_view s)
{
    if (s == "sensor")
        return ConsumerType::Sensor;
    if (s == "thruster")
        return ConsumerType::Thruster;
    return std::nullopt;
}

/**
 * @brief Abstract base class for each power-consuming components on a vessel
 *
 * Subclasses inherit from this and implement:
 *   - drawnCurrent() : their individual current draw at this tick
 *   - update()       : any per-tick internal state update
 *
 * Priority groups for consumers:
 *   1 = safety critical        — never shed by Platform Power Manager
 *   2 = operationally important
 *   3 = mission systems        — default if omitted
 *   4 = non-essential          — shed first
 */
class PowerConsumer {
public:
    struct CreateResult {
        std::shared_ptr<PowerConsumer> consumer;  // nullptr on failure
        ConsumerType type;
    };

    static CreateResult createFromSdf(
        const std::string& consumer_name,
        const std::string& vessel_name,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<spdlog::logger> logger);

    virtual ~PowerConsumer() = default;

    /**
     * @brief Individual current draw of this component (Amp)
     *        Called by Platform Power Manager to sum total bus load
     *        returns 0.0 if !isActive() or if m_voltage <= 0.0
     *        Formula: I = P / V
     *        SensorConsumer  : nominalPowerW() / m_voltage
     *        ThrusterConsumer: (nominalPowerW() * rpm_ratio²) / m_voltage
     */
    virtual float drawnCurrent() const = 0;

    /**
     * @brief called by Platform Power Manager
     *        Subclass updates any internal state that changes
     *        SensorConsumer  : nothing (fixed draw, no internal state to
     * update) ThrusterConsumer: reads current RPM, updates m_rpmRatio
     */
    virtual void update() = 0;

    /**
     * @brief Deactivates this consumer —> called by Platform Power Manager
     * during load shedding or full power loss Subclass should override to add
     * type-specific shutdown behaviour (e.g. ThrusterConsumer sets rpm to zero,
     * SensorConsumer publishes a power_state = OFF notification)
     */
    virtual void deactivate()
    {
        m_active = false;
    }

    /**
     * @brief Activates this consumer
     *        Sets m_active = true
     *        Called by Platform Power Manager
     *        TODO: Platform Power Manager currently has no reactivation logic
     * -> call activate() in shedLoadsIfNeeded() when PowerLevel returns to
     * NORMAL
     */
    virtual void activate()
    {
        m_active = true;
    }

    /**
     * @brief receives the current bus voltage from Platform Power Manager
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
    bool isActive() const
    {
        return m_active;
    }

    // priority group for load shedding, the larger the number, the less
    // critical the system read from SDF priority attribute. default = 3
    int priority() const
    {
        return m_priority;
    }

    // Nominal power draw in Watts —> read from SDF nominal_w
    // used by Platform Power Manager::shedLoadsIfNeeded() for demand estimation
    float nominalPowerW() const
    {
        return m_nominalPowerW;
    }

    const std::string& name() const
    {
        return m_consumer_name;
    }

    const std::string& vesselName() const
    {
        return m_vessel_name;
    }

    void setPriority(const int& priority)
    {
        m_priority = priority;
    }

protected:
    /**
     * @brief only subclasses can instantiate
     *
     * @param name        component name from SDF
     * @param node        shared node owned by Platform Power Manager
     * @param sdf         lotusim_power sdf pointer
     */
    PowerConsumer(
        const std::string& consumer_name,
        const std::string& vessel_name,
        const sdf::ElementPtr& sdf,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<spdlog::logger> logger)
        : m_node(std::move(node))
        , m_logger(std::move(logger))
        , m_consumer_name(std::move(consumer_name))
        , m_vessel_name(std::move(vessel_name))
    {
        m_nominalPowerW = sdf->Get<float>("nominal_w", 1.0f).first;
        m_priority = sdf->Get<int>("priority", 3).first;
    }

    // last voltage pushed by Platform Power Manager via receiveVoltage()
    // used by subclasses in drawnCurrent()
    float m_voltage{0.0f};

    // nominal power draw in Watts, from SDF nominal_w
    float m_nominalPowerW{0.0f};

    int m_priority{3};

    // Shared node
    rclcpp::Node::SharedPtr m_node;

    std::shared_ptr<spdlog::logger> m_logger;

    std::string m_consumer_name;

    std::string m_vessel_name;

    bool m_active{true};
};
}  // namespace lotusim::gazebo