/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "battery_sensor/battery_sensor.hpp"

#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/double.pb.h>

#include <gz/sim/Conversions.hh>
#include <gz/transport/TopicUtils.hh>

#include <chrono>
#include <cmath>
#include <functional>

#include "lotusim_common/common.hpp"

namespace lotusim::sensor {

// ═══════════════════════════════════════════════════════════════════════════
// Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════════════
BatterySensor::BatterySensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity& vessel_entity,
    const gz::sim::Entity& sensor_entity,
    const std::string& parent_name,
    const std::string& sensor_name)
    : CustomSensor(
          logger,
          node,
          vessel_entity,
          sensor_entity,
          parent_name,
          sensor_name)
{
}

BatterySensor::~BatterySensor()
{
    if (m_battery) {
        if (m_consumer_id != -1)
            m_battery->RemoveConsumer(m_consumer_id);
        m_battery->ResetUpdateFunc();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CustomSensorLoad
// ═══════════════════════════════════════════════════════════════════════════
bool BatterySensor::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr elem = _sdf.Element();

    if (elem->HasElement("open_circuit_voltage_constant_coef"))
        m_e0 = elem->Get<double>("open_circuit_voltage_constant_coef");
    if (elem->HasElement("open_circuit_voltage_linear_coef"))
        m_e1 = elem->Get<double>("open_circuit_voltage_linear_coef");

    if (elem->HasElement("capacity"))
        m_c = elem->Get<double>("capacity");
    if (m_c <= 0.0) {
        m_logger->error(
            "BatterySensor [{}]: <capacity> must be > 0",
            m_sensor_name);
        return false;
    }

    m_q0 = m_c;
    if (elem->HasElement("initial_charge")) {
        m_q0 = elem->Get<double>("initial_charge");
        m_q0 = std::max(0.0, std::min(m_q0, m_c));
    }
    m_q = m_q0;

    if (elem->HasElement("resistance"))
        m_r = elem->Get<double>("resistance");

    if (elem->HasElement("smooth_current_tau")) {
        m_tau = elem->Get<double>("smooth_current_tau");
        if (m_tau <= 0.0)
            m_tau = 1.0;
    }

    if (elem->HasElement("fix_issue_225"))
        m_fix_issue_225 = elem->Get<bool>("fix_issue_225");

    if (!elem->HasElement("battery_name") || !elem->HasElement("voltage")) {
        m_logger->error(
            "BatterySensor [{}]: <battery_name> and <voltage> are required",
            m_sensor_name);
        return false;
    }
    m_battery_name = elem->Get<std::string>("battery_name");
    double init_voltage = elem->Get<double>("voltage");

    m_battery =
        std::make_shared<gz::common::Battery>(m_battery_name, init_voltage);
    m_battery->Init();
    m_battery->SetUpdateFunc(std::bind(
        &BatterySensor::OnUpdateVoltage,
        this,
        std::placeholders::_1));

    // Recharge
    if (elem->HasElement("enable_recharge") &&
        elem->Get<bool>("enable_recharge")) {
        if (!elem->HasElement("charging_time")) {
            m_logger->error(
                "BatterySensor [{}]: <charging_time> required when recharge enabled",
                m_sensor_name);
            return false;
        }
        m_t_charge = elem->Get<double>("charging_time");

        const std::string enable_topic =
            gz::transport::TopicUtils::AsValidTopic(
                "/model/" + m_vessel_name + "/battery/" + m_battery_name +
                "/recharge/start");
        const std::string disable_topic =
            gz::transport::TopicUtils::AsValidTopic(
                "/model/" + m_vessel_name + "/battery/" + m_battery_name +
                "/recharge/stop");

        if (!enable_topic.empty() && !disable_topic.empty()) {
            m_gz_node.Advertise(
                enable_topic,
                &BatterySensor::OnEnableRecharge,
                this);
            m_gz_node.Advertise(
                disable_topic,
                &BatterySensor::OnDisableRecharge,
                this);
            if (elem->HasElement("recharge_by_topic")) {
                m_gz_node.Subscribe(
                    enable_topic,
                    &BatterySensor::OnEnableRecharge,
                    this);
                m_gz_node.Subscribe(
                    disable_topic,
                    &BatterySensor::OnDisableRecharge,
                    this);
            }
        }
    }

    // Static power load
    if (elem->HasElement("power_load")) {
        m_initial_power_load = elem->Get<double>("power_load");
        m_consumer_id = m_battery->AddConsumer();
        m_battery->SetPowerLoad(m_consumer_id, m_initial_power_load);
    }

    if (elem->HasElement("start_draining"))
        m_start_draining = elem->Get<bool>("start_draining");

    // Extra draining topics
    if (elem->HasElement("power_draining_topic")) {
        auto sdf_elem = elem->FindElement("power_draining_topic");
        while (sdf_elem) {
            m_gz_node.SubscribeRaw(
                sdf_elem->Get<std::string>(),
                std::bind(
                    &BatterySensor::OnBatteryDrainingMsg,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
            sdf_elem = sdf_elem->GetNextElement("power_draining_topic");
        }
    }
    if (elem->HasElement("stop_power_draining_topic")) {
        auto sdf_elem = elem->FindElement("stop_power_draining_topic");
        while (sdf_elem) {
            m_gz_node.SubscribeRaw(
                sdf_elem->Get<std::string>(),
                std::bind(
                    &BatterySensor::OnBatteryStopDrainingMsg,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
            sdf_elem = sdf_elem->GetNextElement("stop_power_draining_topic");
        }
    }

    m_soc = m_q / m_c;

    // gz transport publishers
    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(50);

    const std::string state_topic =
        gz::transport::TopicUtils::AsValidTopic(
            "/model/" + m_vessel_name + "/battery/" + m_battery_name +
            "/state");
    const std::string power_topic =
        gz::transport::TopicUtils::AsValidTopic(
            "/model/" + m_vessel_name + "/battery/power");

    m_state_pub =
        m_gz_node.Advertise<gz::msgs::BatteryState>(state_topic, opts);
    m_power_pub = m_gz_node.Advertise<gz::msgs::Double>(power_topic, opts);

    // Native ROS publisher (relative topic → resolved under the world namespace,
    // e.g. /<world>/<vessel>/battery/state).  This is what remote ROS-only agents
    // subscribe to; gz transport does not cross machines, so the gz publisher
    // above is not enough by itself.  TRANSIENT_LOCAL so a late subscriber still
    // gets the last state (matches the previous agent-side bridge QoS).
    m_ros_state_pub =
        m_ros_node->create_publisher<sensor_msgs::msg::BatteryState>(
            m_vessel_name + "/battery/state",
            rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

    m_logger->info(
        "BatterySensor [{}]: battery '{}' ready — cap={} Ah, V0={}",
        m_sensor_name,
        m_battery_name,
        m_c,
        init_voltage);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// UpdateSensor — called every sim tick by LotusimSensorPlugin::PostUpdate
// ═══════════════════════════════════════════════════════════════════════════
bool BatterySensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    // One-time subscription to vessel commands (world name known only here)
    if (!m_subscribed) {
        const std::string world_name = lotusim::common::getWorldName(_ecm);
        const std::string cmd_topic =
            "/" + world_name + "/vessel_cmd_array";

        m_vessel_cmd_sub =
            m_ros_node->create_subscription<lotusim_msgs::msg::VesselCmdArray>(
                cmd_topic,
                rclcpp::QoS(10),
                [this](
                    const lotusim_msgs::msg::VesselCmdArray::SharedPtr msg) {
                    for (const auto& cmd : msg->cmds) {
                        if (cmd.vessel_name != m_vessel_name)
                            continue;
                        try {
                            const auto& s = cmd.cmd_string;
                            auto pos = s.find("(rpm)\":");
                            if (pos != std::string::npos) {
                                auto val_start =
                                    s.find_first_of("-0123456789.", pos + 7);
                                auto val_end = s.find_first_not_of(
                                    "-0123456789.",
                                    val_start);
                                double rpm = std::stod(
                                    s.substr(val_start, val_end - val_start));
                                std::unique_lock<std::shared_mutex> lock(
                                    m_rpm_mutex);
                                m_latest_rpm = std::abs(rpm);
                                m_start_draining = rpm > 0.0;
                            }
                        } catch (...) {
                            m_logger->warn(
                                "BatterySensor [{}]: failed to parse cmd_string",
                                m_sensor_name);
                        }
                    }
                });

        m_logger->info(
            "BatterySensor [{}]: subscribed to [{}]",
            m_sensor_name,
            cmd_topic);
        m_subscribed = true;
    }

    // Drain the ROS 2 callback queue so RPM updates are processed
    rclcpp::spin_some(m_ros_node);

    if (_info.paused)
        return true;

    if (!m_start_draining && !m_start_charging)
        return true;

    // Print drain time in minutes (debug)
    int sim_sec = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime)
            .count());
    if (m_drain_start_time == -1)
        m_drain_start_time = sim_sec;
    int drain_min = (sim_sec - m_drain_start_time) / 60;
    if (drain_min != m_last_print_time) {
        m_last_print_time = drain_min;
        m_logger->debug(
            "BatterySensor [{}]: {} min elapsed",
            m_sensor_name,
            drain_min);
    }

    m_step_size = _info.dt;

    double dt =
        std::chrono::duration_cast<std::chrono::nanoseconds>(m_step_size)
            .count() *
        1e-9;
    if (m_tau < dt)
        m_tau = dt;

    // Run battery physics (calls OnUpdateVoltage using m_total_power from last tick)
    if (m_battery)
        m_battery->Update();

    // Recompute power from RPM for the next tick
    {
        std::shared_lock<std::shared_mutex> lock(m_rpm_mutex);
        double rpm = m_latest_rpm;
        if (rpm > 0.0) {
            double coeff =
                9e-23 * std::pow(rpm, 6) + 5e-18 * std::pow(rpm, 5) +
                1e-13 * std::pow(rpm, 4) - 5e-11 * std::pow(rpm, 3) +
                4e-7 * std::pow(rpm, 2) + 6e-5 * rpm;
            m_total_power = (m_battery->Voltage() * coeff) / 0.9;
        } else {
            m_total_power = 0.0;
        }
    }

    // Publish battery state on gz transport
    if (m_state_pub) {
        gz::msgs::BatteryState state_msg;
        state_msg.mutable_header()->mutable_stamp()->CopyFrom(
            gz::sim::convert<gz::msgs::Time>(_info.simTime));
        state_msg.set_voltage(m_battery->Voltage());
        state_msg.set_current(m_ismooth);
        state_msg.set_charge(m_q);
        state_msg.set_capacity(m_c);
        state_msg.set_percentage(m_fix_issue_225 ? m_soc * 100.0 : m_soc);

        if (m_start_charging)
            state_msg.set_power_supply_status(gz::msgs::BatteryState::CHARGING);
        else if (m_start_draining)
            state_msg.set_power_supply_status(
                gz::msgs::BatteryState::DISCHARGING);
        else if (m_soc > 0.9)
            state_msg.set_power_supply_status(gz::msgs::BatteryState::FULL);
        else
            state_msg.set_power_supply_status(
                gz::msgs::BatteryState::NOT_CHARGING);

        m_state_pub.Publish(state_msg);

        // Mirror the same state onto ROS for remote (ROS-only) consumers.
        if (m_ros_state_pub) {
            sensor_msgs::msg::BatteryState ros_msg;
            ros_msg.header =
                lotusim::common::generateHeaderMessage(_info.simTime);
            ros_msg.voltage = m_battery->Voltage();
            ros_msg.current = m_ismooth;
            ros_msg.charge = m_q;
            ros_msg.capacity = m_c;
            ros_msg.design_capacity = m_c;
            ros_msg.percentage = m_fix_issue_225 ? m_soc * 100.0 : m_soc;
            ros_msg.present = true;

            if (m_start_charging)
                ros_msg.power_supply_status =
                    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            else if (m_start_draining)
                ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::
                    POWER_SUPPLY_STATUS_DISCHARGING;
            else if (m_soc > 0.9)
                ros_msg.power_supply_status =
                    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
            else
                ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::
                    POWER_SUPPLY_STATUS_NOT_CHARGING;

            m_ros_state_pub->publish(ros_msg);
        }
    }

    if (m_power_pub) {
        gz::msgs::Double power_msg;
        power_msg.set_data(m_total_power);
        m_power_pub.Publish(power_msg);
    }

    m_last_measurement_time = _info.simTime;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// OnUpdateVoltage — called by gz::common::Battery::Update()
// ═══════════════════════════════════════════════════════════════════════════
double BatterySensor::OnUpdateVoltage(const gz::common::Battery* _battery)
{
    if (!_battery)
        return 0.0;
    if (std::fabs(_battery->Voltage()) < 1e-3 && !m_start_charging)
        return 0.0;
    if (m_soc < 0.0 && !m_start_charging)
        return _battery->Voltage();

    auto prev_soc_int = static_cast<int>(m_soc * 100);

    double dt =
        std::chrono::duration_cast<std::chrono::nanoseconds>(m_step_size)
            .count() *
        1e-9;
    double k = dt / m_tau;

    // Use RPM-based power; fall back to static power loads when idle
    double effective_power = m_total_power;
    if (effective_power <= 0.0 && m_start_draining) {
        for (const auto& load : _battery->PowerLoads())
            effective_power += load.second;
    }
    m_iraw = effective_power / _battery->Voltage();

    // Charging current
    if (m_t_charge > 0.0 && m_start_charging && m_soc < 0.9)
        m_iraw -= m_c / m_t_charge;

    m_ismooth = m_ismooth + k * (m_iraw - m_ismooth);

    if (!m_fix_issue_225) {
        if (m_i_list.size() >= 100) {
            m_i_list.pop_front();
            m_dt_list.pop_front();
        }
        m_i_list.push_back(m_ismooth);
        m_dt_list.push_back(dt);
    }

    m_q = m_q - (dt * m_ismooth / 3600.0);

    double voltage =
        m_e0 + m_e1 * (1.0 - m_q / m_c) - m_r * m_ismooth;

    if (m_fix_issue_225) {
        m_soc = m_q / m_c;
    } else {
        double isum = 0.0;
        for (std::size_t i = 0; i < m_i_list.size(); ++i)
            isum += m_i_list[i] * m_dt_list[i] / 3600.0;
        m_soc = m_soc - isum / m_c;
    }

    auto soc_int = static_cast<int>(m_soc * 100);
    if (soc_int % 10 == 0 && soc_int != prev_soc_int) {
        m_logger->debug(
            "Battery [{}]: {}%, V={}",
            m_battery_name,
            soc_int,
            voltage);
    }
    if (m_soc < 0.0 && !m_drain_printed) {
        m_logger->warn(
            "BatterySensor [{}]: {} is out of battery",
            m_sensor_name,
            m_vessel_name);
        m_drain_printed = true;
    }

    return voltage;
}

// ═══════════════════════════════════════════════════════════════════════════
// gz transport callbacks
// ═══════════════════════════════════════════════════════════════════════════
void BatterySensor::OnEnableRecharge(const gz::msgs::Boolean&)
{
    m_start_charging = true;
}

void BatterySensor::OnDisableRecharge(const gz::msgs::Boolean&)
{
    m_start_charging = false;
}

void BatterySensor::OnBatteryDrainingMsg(
    const char*,
    size_t,
    const gz::transport::MessageInfo&)
{
    m_start_draining = true;
}

void BatterySensor::OnBatteryStopDrainingMsg(
    const char*,
    size_t,
    const gz::transport::MessageInfo&)
{
    m_start_draining = false;
}

}  // namespace lotusim::sensor
