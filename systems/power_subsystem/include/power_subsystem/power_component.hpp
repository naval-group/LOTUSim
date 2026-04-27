/*
 * Copyright (c) 2025 Naval Group
 * SPDX-License-Identifier: EPL-2.0
 */

#pragma OneTimeChange

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace lotusim::power_subsystem{
 /**
 * @brief Abstract base class for all power-consuming components in the vessel.
 *
 * PowerComponent provides a common interface that each power consumer
 * (thruster, sensor array ...) must implement. It subscribes to battery state
 * topics published and keeps a local cached copy of the latest current and voltage 
 * readings so that subclasses can react to battery conditions without 
 * subscribing independently.
 */

class PowerComponent {
public:
    
    struct BatteryState{
        // To read the battery topics
        float current{0.0f};  // <[A] positive = discharging
        float voltage{0.0f};  // <[V]
    }

    /**
    * @param node                shared ROS 2 node (must outlive this object)
    * @param component_name      identifier used in log messages
    * @param current_topic       topic name
    *                            sensor_msgs/BatteryState.
    */
    explicit PowerComponent(
        rclcpp::Node::SharedPtr node,
        const std::string& component_name,
        const std::string& current_topic,   // like "/hectate/OUT_I" but let it be defined by the node
        const std::string& voltage_topic
    );

    virtual ~PowerComponent() = default;

    // subclasses must implement this - compute and return the power consumption [W]
    [[nodiscard("Use the returned power consumption value")]] virtual float power_consume() const = 0;  //probably a bug if its returned value isnt being used

    // return the latest battery state 
    [[nodiscard("Use the returned battery state")]] BatteryState state() const;

    // to enable or disable this component
    // subclasses must respect this flag
    // true -> component allowed to draw power
    void set_active(bool on);

    // return if this component is allowed to draw power
    [[nodiscard("Use the returned active status of the component")]] bool is_active() const;

    const std::string & component_name() const{
        return component_name_;
    }

protected:
    // ROS2 node shared by all components in this subsystem - one for each vessel spawned
    rclcpp::Node::SharedPtr node_;

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
    std::atomic<float> state_voltage_{0.0f}
    // if this component is allowed to consume power
    std::atomic<bool> active_{true};
};
} // namespace lotusim::power_subsystem