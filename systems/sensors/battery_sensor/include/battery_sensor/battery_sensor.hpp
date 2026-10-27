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

#include <gz/msgs/boolean.pb.h>

#include <gz/common/Battery.hh>
#include <gz/transport/Node.hh>

#include <deque>
#include <shared_mutex>

#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_msgs/msg/vessel_cmd_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace lotusim::sensor {

/**
 * @brief Battery sensor for LOTUSim.
 *
 * Simulates a linear battery model driven by RPM commands received on the
 * vessel_cmd_array ROS 2 topic.  Battery state is published to Gazebo
 * transport so it remains compatible with other gz plugins that watch
 * /model/<vessel>/battery/<name>/state.
 *
 * SDF params (inside <sensor type="custom" gz:type="battery">):
 *   <battery_name>             string  — required
 *   <voltage>                  double  — initial open-circuit voltage (V), required
 *   <open_circuit_voltage_constant_coef>  double  (e0)
 *   <open_circuit_voltage_linear_coef>    double  (e1)
 *   <capacity>                 double  — Ah, required and > 0
 *   <initial_charge>           double  — Ah, defaults to capacity
 *   <resistance>               double  — internal resistance (Ohm)
 *   <smooth_current_tau>       double  — low-pass time constant (s)
 *   <fix_issue_225>            bool
 *   <power_load>               double  — static baseline power draw (W)
 *   <start_draining>           bool
 *   <enable_recharge>          bool
 *   <charging_time>            double  — hours for full charge (required if recharge enabled)
 *   <recharge_by_topic>        bool
 *   <power_draining_topic>     string  — gz topic to start draining (repeatable)
 *   <stop_power_draining_topic> string — gz topic to stop draining (repeatable)
 */
class BatterySensor : public CustomSensor {
public:
    BatterySensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~BatterySensor() override;

    bool CustomSensorLoad(const sdf::Sensor& _sdf) override;

    bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

private:
    double OnUpdateVoltage(const gz::common::Battery* _battery);

    void OnEnableRecharge(const gz::msgs::Boolean&);
    void OnDisableRecharge(const gz::msgs::Boolean&);
    void OnBatteryDrainingMsg(
        const char*, size_t, const gz::transport::MessageInfo&);
    void OnBatteryStopDrainingMsg(
        const char*, size_t, const gz::transport::MessageInfo&);

    // ── Physics params (from SDF) ─────────────────────────────────────────
    double m_e0{0.0};
    double m_e1{0.0};
    double m_c{0.0};
    double m_q0{0.0};
    double m_r{0.0};
    double m_tau{1.0};
    double m_t_charge{0.0};
    double m_initial_power_load{0.0};
    bool m_fix_issue_225{false};

    // ── Runtime state ─────────────────────────────────────────────────────
    double m_q{0.0};
    double m_iraw{0.0};
    double m_ismooth{0.0};
    double m_soc{1.0};
    double m_total_power{0.0};
    double m_latest_rpm{0.0};

    bool m_start_draining{false};
    bool m_start_charging{false};
    bool m_drain_printed{false};
    bool m_subscribed{false};

    int m_drain_start_time{-1};
    int m_last_print_time{-1};

    std::deque<double> m_i_list;
    std::deque<double> m_dt_list;
    std::chrono::steady_clock::duration m_step_size{};
    std::shared_mutex m_rpm_mutex;

    // ── gz::common::Battery ───────────────────────────────────────────────
    std::string m_battery_name;
    gz::common::BatteryPtr m_battery;
    int32_t m_consumer_id{-1};

    // ── gz transport ──────────────────────────────────────────────────────
    gz::transport::Node m_gz_node;
    gz::transport::Node::Publisher m_state_pub;
    gz::transport::Node::Publisher m_power_pub;

    // ── ROS 2 ─────────────────────────────────────────────────────────────
    rclcpp::Subscription<lotusim_msgs::msg::VesselCmdArray>::SharedPtr
        m_vessel_cmd_sub;
    // Native ROS publisher so the battery state reaches remote agents (which run
    // ROS only, no Gazebo) over DDS — mirrors how ais_sensor publishes directly.
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
        m_ros_state_pub;
};

}  // namespace lotusim::sensor
