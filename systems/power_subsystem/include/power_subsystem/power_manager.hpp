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
 *   1. Sum drawCurrent() from all active consumers -> totalCurrent
 *   2. Sum availablePowerW() from all generators -> genSupply
 *   3. If genSupply() >= demand:  generators cover load, surplus -> selectChargeTarget()->receiveCharge()
 *      else: shortfall drawn from battery via receiveLoad()
 *   4. If active battery soc < m_socMinThreshold -> updateActiveProvider()
 *         if all batteries depleted -> deactivate all consumers
 *   5. shedLoadsIfNeeded() -> based on combined availablePowerW() vs demand
 *   6. Push voltage to all consumers via receiveVoltage() + update()
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
     * @brief Walks all <battery> tags in SDF order and populates m_providers
     *        Also populates m_generators and m_batteries
     *        SDF order = priority order
     */
    void parsePowerProviders(const sdf::ElementPtr _sdf);

    /**
     * @brief Walks all tags with power_type attribute and populates m_consumers
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
    PowerProvider* selectChargeTarget() const;

    /**
     * @brief Sheds non-essential consumers when combined available power
     *        falls below total demand + safety margin
     *        Never touches priority 1
     *        Trigger: sum(availablePowerW()) < totalDemand * (1 + m_socMinThreshold)
     *
     *        TODO: add reactivation logic when power margin recovers
     */
    void shedLoadsIfNeeded();

    gz::sim::Entity m_entity;

    std::shared_ptr<spdlog::logger> m_logger;

    std::string m_vesselName;

    // Single ROS2 node shared by all PowerProviders and PowerConsumers of
    // this vessel. Created in Configure(), destroyed with PowerManager
    rclcpp::Node::SharedPtr m_node;

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

    // soc threashold below which shedding begins
    // read from <battery> SDF tag as soc_min
    float m_socMinThreshold{0.1f};
};
} //namespace lotusim::gazebo