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

#include <array>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "lotusim_common/logger.hpp"
#include "lotusim_msgs/msg/power_status.hpp"
#include "power_subsystem/power_consumer/power_consumer.hpp"
#include "power_subsystem/power_provider/battery.hpp"
#include "power_subsystem/power_provider/generator.hpp"
#include "power_subsystem/power_provider/power_provider.hpp"

namespace lotusim::gazebo {

enum class PlatformPowerManagerType
{
    DEFAULT
};

static constexpr std::
    array<std::pair<std::string_view, PlatformPowerManagerType>, 1>
        kPlatformPowerManagerTypeEntries{
            {{"default", PlatformPowerManagerType::DEFAULT}}};

inline PlatformPowerManagerType platformPowerManagerTypeFromString(
    const std::string& s)
{
    for (const auto& [str, val] : kPlatformPowerManagerTypeEntries)
        if (str == s)
            return val;
    throw std::invalid_argument("Unknown PlatformPowerManagerType: " + s);
}

inline std::string platformPowerManagerTypeToString(PlatformPowerManagerType t)
{
    for (const auto& [str, val] : kPlatformPowerManagerTypeEntries)
        if (val == t)
            return std::string(str);
    throw std::invalid_argument("Unhandled PlatformPowerManagerType");
}

/**
 * @brief per-vessel power accounting logic. Created by world-level PowerManager
 *        when a vessel is spawned.
 *
 * Populated at Configure() time:
 *   - m_providers  : priority-ordered PowerProvider list (batteries)
 *   - m_consumers  : PowerConsumer list
 *   - m_batteries  : Pnon-owning filtered view into m_providers
 *
 * On each sim tick, Update() runs:
 *   1. EachNew / EachRemoved -> notify consumers
 *   2. Sum drawCurrent() from all active consumers -> total_current
 *   3. load distribution logic
 *   4. Push voltage to all consumers via receiveVoltage() + update()
 *
 * Battery declared as a <link> with <lotusim_power> tag in vessel SDF
 *
 * SDF format:
 *   <model name="dtmb">
 *     <link name="main_battery">
 *       <lotusim_power>
 *         <type>simple_battery</type>
 *         <capacity_ah>100</capacity_ah>
 *         <initial_soc>1.0</initial_soc>
 *         <voltage_min>36.0</voltage_min>
 *         <voltage_nominal>48.0</voltage_nominal>
 *       </lotusim_power>
 *     </link>
 *   </model>
 */

class PlatformPowerManagerBase {
public:
    /**
     * @param m_vessel_entity  Gazebo model entity for this vessel
     * @param m_vessel_name    vessel name
     * @param node             shared ROS2 node
     * @param _ecm             Gazebo ECM
     */
    PlatformPowerManager(
        gz::sim::Entity m_vessel_entity,
        const std::string& m_vessel_name,
        rclcpp::Node::SharedPtr node,
        sdf::ElementPtr sdfptr,
        gz::sim::EntityComponentManager& _ecm);

    ~PlatformPowerManager();

    /**
     * @brief Called every Gazebo tick by PowerManager world plugin
     */
    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief read sensors as declared through ECM -> need more time
     */
    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm);

    // for logging and topic
    const std::string& vesselName() const
    {
        return m_vessel_name;
    }

    float activeBusVoltage() const;

    static std::unique_ptr<PlatformPowerManagerBase> create(
        PlatformPowerManagerType type,
        gz::sim::Entity vessel_entity,
        const std::string& vessel_name,
        rclcpp::Node::SharedPtr node,
        sdf::ElementPtr sdfptr,
        gz::sim::EntityComponentManager& ecm);

private:
    /**
     * @brief Traverse sdf of the vessel and finds <lotusim_power> tag
     *        SDF order = priority order
     */
    bool initPowerProvider(sdf::ElementPtr sdfptr);

    /**
     * @brief for each sensor with a type attribute, constructs the
     *        appropriate PowerConsumer subclass
     */
    bool initPowerConsumers(
        sdf::ElementPtr sdfptr,
        gz::sim::EntityComponentManager& _ecm);

    void publishPowerStatus();

protected:
    /**
     * @brief For user to override when they define their own power manager
     *
     * @param dt
     */
    virtual void handlePowerUpdate(
        float dt,
        gz::sim::EntityComponentManager& _ecm,
        const gz::sim::UpdateInfo& _info) = 0;

protected:
    gz::sim::Entity m_vessel_entity;
    std::string m_vessel_name;
    rclcpp::Node::SharedPtr m_node;
    std::shared_ptr<spdlog::logger> m_logger;

    std::vector<std::shared_ptr<PowerConsumer>> m_consumers;
    std::vector<std::shared_ptr<Battery>> m_batteries;
    std::vector<std::shared_ptr<Generator>> m_generators;
    std::vector<std::shared_ptr<PowerProvider>> m_providers;

    bool m_all_batteries_depleted{false};
    int m_active_battery_index{-1};
    int m_active_generators_index{-1};

    rclcpp::Publisher<lotusim_msgs::msg::PowerStatus>::SharedPtr
        m_power_status_pub;
    rclcpp::TimerBase::SharedPtr m_power_status_timer;
};
}  // namespace lotusim::gazebo