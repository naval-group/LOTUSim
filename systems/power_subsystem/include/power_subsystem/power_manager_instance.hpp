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

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <sdf/Element.hh>
#include <rclcpp/rclcpp.hpp>

#include "lotusim_common/logger.hpp"
#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/power_consumer.hpp"
#include "power_subsystem/battery.hpp"
#include "power_subsystem/generator.hpp"

namespace lotusim::gazebo{

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
 *   3. push total_current to active battery via receiveLoad() (generator path to be implemented later)
 *   (3. If genSupply() >= demand:  generators cover load, surplus -> selectChargeTarget()->receiveCharge()
 *      else: shortfall drawn from battery via receiveLoad())
 *   4. Read PowerLevel from active battery:
 *        DEPLETED : log info + switch to next battery, deactivate all if none left
 *        CRITICAL : log warning + shed priority 3 and below
 *        WARN     : log warning + shed priority 4
 *        NORMAL   : no action
 *   5. Push voltage to all consumers via receiveVoltage() + update()
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

class PowerManagerInstance {
public:
    /**
     * @param m_vessel_entity  Gazebo model entity for this vessel
     * @param m_vessel_name    vessel name
     * @param node             shared ROS2 node
     * @param _ecm             Gazebo ECM
     */
     PowerManagerInstance(
        gz::sim::Entity m_vessel_entity,
        const std::string& m_vessel_name,
        rclcpp::Node::SharedPtr node,
        gz::sim::EntityComponentManager& _ecm);

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
    const std::string& vesselName() const { return m_vessel_name; }

private:
    // ----------------------------------------------------------------
    // helpers 
    // ----------------------------------------------------------------

    /**
     * @brief look directly at the SDF
     */
    static sdf::ElementPtr findRawSensorElement(
        const sdf::ElementPtr& modelEl,
        const std::string& sensorName);
        
    /**
     * @brief Walks links of the vessel and finds <lotusim_power> tag 
     *        SDF order = priority order
     */
    bool parsePowerProviders(gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief for each sensor with a power_type attribute, constructs the
     *        appropriate PowerConsumer subclass
     */
    bool parsePowerConsumers(gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief Walks m_batteries from m_activeBatteryIndex forward
     *        updates m_activeBatteryIndex to first non-depleted battery
     *        returns false if all batteries are depleted
     */
    bool updateActiveProvider();

    /**
     * @brief Sheds consumers based on PowerLevel
     *        TODO: add reactivation logic when power margin recovers
     */
    void shedLoadsIfNeeded(PowerLevel level);

    /**
     * @brief Returns true if the Gazebo entity name matches consumerName
     */
    bool matchesEntity(
        const std::string& consumerName,
        const gz::sim::Entity& _entity,
        gz::sim::EntityComponentManager& _ecm) const;

    /**
     * @brief Returns true if the link's parent matches the vessel
     */
    bool isChildOfVessel(
        const gz::sim::Entity& /*_entity*/,
        const gz::sim::Entity& parentEntity,
        gz::sim::EntityComponentManager& _ecm) const;

    /**
     * @brief to give time to gazebo to load in the sensors
     */
    bool m_consumers_parsed{false};

    //
    //
    //
    gz::sim::Entity m_vessel_entity;
    std::string m_vessel_name;
    rclcpp::Node::SharedPtr m_node;
    std::shared_ptr<spdlog::logger> m_logger;

    //priority ordered list of power providers
    std::vector<std::unique_ptr<PowerProvider>> m_providers;

    // all consumers
    std::vector<std::unique_ptr<PowerConsumer>> m_consumers;

    // Non-owning filtered views
    std::vector<Battery*> m_batteries;
    std::vector<Generator*> m_generators;

    //std::unordered_map<gz::sim::Entity, std::unique_ptr<PowerManagerInstance>> m_vessels;

    //index into m_batteries of the currently active battery
    int m_activeBatteryIndex{0};
};
} //namespace lotusim::gazebo