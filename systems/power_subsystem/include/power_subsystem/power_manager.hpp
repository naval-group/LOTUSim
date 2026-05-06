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

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>

namespace lotusim::gazebo{

// forward declarations
class PowerProvider;
class PowerConsumer;

/**
 * @brief This is a plugin for interfacing with external power subsystem
 * One instance is created per vessel. It owns:
 * - m_providers : priority-ordered list of PowerProvider (Battery, Generator). SDF declaration order = priority order
 * - m_consumers : list of PowerConsumer (SensorPowerConsumer, ThrusterPowerConsumer) 
 *
 * Populated at Configure() time:
 *   - m_generators : PowerProvider* where isGenerator() == true
 *   - m_batteries  : PowerProvider* where canReceiveCharge() == true
 *
 * On each sim tick, Update() runs the following logic:
 *   1. EachNew / EachRemoved -> notify consumers 
 *   2. Sum drawCurrent() from all active consumers -> totalCurrent
 *   3. push totalCurrent to active battery via receiveLoad() (generator path to be implemented later)
 *   (3. If genSupply() >= demand:  generators cover load, surplus -> selectChargeTarget()->receiveCharge()
 *      else: shortfall drawn from battery via receiveLoad())
 *   4. Read PowerLevel from active battery:
 *        DEPLETED : log info + switch to next battery, deactivate all if none left
 *        CRITICAL : log warning + shed priority 3 and below
 *        WARN     : log warning + shed priority 4
 *        NORMAL   : no action
 *   5. Push voltage to all consumers via receiveVoltage() + update()
 * 
 * SDF format:
 *   <provider name="main_battery"  type="nuclear_battery"
 *             capacity_ah="200" initial_soc="1.0" voltage_min="40.0"/>
 *
 *   <sensor name="ais_sensor" type="custom" gz:type="ais"
 *           power_type="sensor" nominal_w="5.0" priority="3">
 *     ...
 *   </sensor>
 *   <thruster name="bow_thruster"
 *             nominal_w="800.0" max_rpm="1200" priority="2"/>
 *
 * m_node is created here and passed into every providers and consumers
 */

class PowerManager : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemUpdate{
public:
    PowerManager();
    ~PowerManager() override;

     /**
     * @brief parses SDF, creates shared ROS2 node, builds m_providers,
     *        m_consumers, m_generators and m_batteries
     */
    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf, 
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;
    
    /**
     * @brief runs the full power loop 
     */
    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief Walks all <provider> tags in SDF order and populates 
     *        m_providers and m_batteries
     *        SDF order = priority order
     */
    void parsePowerProviders(const sdf::ElementPtr _sdf);

    /**
     * @brief Walks <sensor> and <thrusters> attribute and populates m_consumers
     */
    void parsePowerConsumers(
        const sdf::ElementPtr _sdf,
        gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief Walks m_batteries from m_activeBatteryIndex forward
     *        updates m_activeBatteryIndex to first non-depleted battery
     *        returns false if all batteries are depleted
     */
    bool updateActiveProvider();

    /**
     * @brief Returns the battery to charge
     *        Priority: first depleted battery, else current active battery
     *        Returns nullptr if m_batteries is empty (generator-only vessel,
     *        or no providers configured)
     */
    PowerProvider* selectChargeTarget() const; // leaving alone for now

    /**
     * @brief Sheds consumers based on PowerLevel
     *        TODO: add reactivation logic when power margin recovers
     */
    void shedLoadsIfNeeded(PowerLevel level);

    /**
     * @brief Used by EachNew / EachRemoved callbacks to route topology
     *        events to the correct consumer
     */
    bool matchesEntity(
        const std::string& consumerName,
        const gz::sim::Entity& _entity,
        gz::sim::EntityComponentManager& _ecm) const;


    gz::sim::Entity m_entity;

    std::shared_ptr<spdlog::logger> m_logger;

    std::string m_vesselName;

    // Single ROS2 node shared by all PowerProviders and PowerConsumers of
    // this vessel. Created in Configure(), destroyed with PowerManager
    rclcpp::Node::SharedPtr m_node;

    gz::sim::EntityComponentManager* m_ecm{nullptr};

    //priority ordered list of power providers
    std::vector<std::unique_ptr<PowerProvider>> m_providers;

    // list of power consumers: one entry per component
    std::vector<std::unique_ptr<PowerConsumer>> m_consumers;

    // Subset of m_providers where isGenerator() == true
    std::vector<PowerProvider*> m_generators;
 
    // Subset of m_providers where canReceiveCharge() == true
    std::vector<PowerProvider*> m_batteries;

    //index into m_providers of the currently active provider
    // updated by activeProviderIndex()
    int m_activeBatteryIndex{0};
};
} //namespace lotusim::gazebo