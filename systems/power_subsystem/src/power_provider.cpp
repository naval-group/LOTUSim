/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

 #include "power_subsystem/power_provider.hpp"
 #include "rclcpp/logging.hpp"

 namespace lotusim::power_subsystem{

    PowerProvider::PowerProvider(
        rclcpp::Node::SharedPtr node,
        const std::string& component_name,
        const std::string& current_topic,
        const std::string& voltage_topic
    ) : node_(std::move(node)),
        component_name_(component_name)
    {
        // splitting current and voltage as currently 2 different topics by fmu
        current_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            current_topic,
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                on_current(msg);
            });

        RCLCPP_INFO( 
            node_->get_logger(),
            "[PowerProvider] '%s' subscribed to current on '%s'",
            component_name_.c_str(),
            current_topic.c_str()
        );

        voltage_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            voltage_topic,
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                on_voltage(msg);
            });

        RCLCPP_INFO( 
            node_->get_logger(),
            "[PowerProvider] '%s' subscribed to voltage on '%s'",
            component_name_.c_str(),
            voltage_topic.c_str()
        );
    }
    
    void PowerProvider::on_current(const std_msgs::msg::Float64::SharedPtr msg){
        //update the cached readongs
        state_current_.store(static_cast<float>(msg->data));

        RCLCPP_DEBUG(
            node_->get_logger(),
            "[PowerProvider] '%s' battery update – current: %.3f A",
            component_name_.c_str(),
            msg->data
        );
    }

    void PowerProvider::on_voltage(const std_msgs::msg::Float64::SharedPtr msg){
        state_voltage_.store(static_cast<float>(msg->data));

        RCLCPP_DEBUG(
            node_->get_logger(),
            "[PowerProvider] '%s' battery update – voltage: %.3f V",
            component_name_.c_str(),
            msg->data
        );
    }

    // info for both
    PowerProvider::BatteryState PowerProvider::state() const{
        return BatteryState{
            state_current_.load(),
            state_voltage_.load()
        };
    }

    void PowerProvider::set_active(bool on){
        const bool was_active = active_.exchange(on); //writes the value "on" into active_ + returns what was in active_

        // if the status change, then publish
        if (was_active != on) {
            RCLCPP_INFO(
                node_->get_logger(),
                "[PowerProvider] '%s' %s",
                component_name_.c_str(),
                on ? "activated" : "deactivated (load shedding)");
        }
    }

    // reads and returns the value of active_
    // to call inside power_consume()
    bool PowerProvider::is_active() const{
        return active_.load();
    }

 }