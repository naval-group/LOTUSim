/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_manager.hpp"

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/common/Console.hh>

// abstract classes
#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/power_consumer.hpp"

// PowerProvider subclasses
#include "power_subsystem/battery.hpp"
//#include "generator_provider_base.hpp"

// PowerConsumer subclasses
#include "power_subsystem/sensor_power_consumer.hpp"
#include "power_subsystem/thruster_power_consumer.hpp"

namespace lotusim::gazebo{

PowerManager::PowerManager();

PowerManager::~PowerManager()
{
    m_logger->info(
        "PowerManager::~PowerManager: PowerManager successfully shutdown.");
}

void PowerManager::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _eventMgr)
{
    m_entity = _entity;
    m_ecm = &_ecm;

    m_world_name = lotusim::common::getWorldName(_ecm);
    m_logger = logger::createConsoleAndFileLogger(
        "power_manager",
        m_world_name + "_power_manager.txt");

    m_vesselName = _ecm.ComponentData<gz::sim::components::Name>(m_entity);
    m_logger->info("PowerManager: configuring vessel {}", m_vesselName);

    // ── ROS2 node ──────────────────────────────────────────────────────
    // one node per vessel
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    m_node = rclcpp::Node::make_shared("power", m_vesselName);

    // ── Build provider and consumer lists ─────────────────────────────
    // m_generators and m_batteries populated inside 
    // parsePowerProviders() alongside m_providers
    parsePowerProviders(_sdf);
    parsePowerConsumers(_sdf, _ecm);

    m_logger->info("PowerManager: Vessel {} has {} providers and {} consumers", 
        m_vesselName, m_providers.size(), m_consumers.size());
}

void PowerManager::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // dt in seconds —> needed by receiveLoad() for fuel/soc integration
    const float dt = static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    // ── Guard ─────────────────────────────────────────────────────────
    if (m_providers.empty()){
        m_logger->info("PowerManager: Vessel {} has no power providers configured.", m_vessel_name);
        return;
    }

    // ── Step 1: topology change callbacks ─────────────────────────────
    // EachNew fires when a CustomSensor component appears in the ECM
    // EachRemoved fires when one is removed
    // Thrusters: no component equivalent yet 
    //  ---------------- is it the best way??
    _ecm.EachNew<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::CustomSensor*) -> bool
        {
            for (auto& consumer : m_consumers) {
                if (matchesEntity(consumer->name(), _entity, _ecm)) {
                    consumer->eachNew();
                }
            }
            return true;
        }
    );
 
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::CustomSensor*) -> bool
        {
            for (auto& consumer : m_consumers) {
                if (matchesEntity(consumer->name(), _entity, _ecm)) {
                    consumer->eachDelete();
                }
            }
            return true;
        }
    );


    // ── Step 2: sum consumer demand ───────────────────────────────────
    //each consumer calculates its own draw. PowerManager owns the sum
    float total_current = 0.0f;
    for(const auto& consumer : m_consumers){
        if(consumer->isActive()){
            total_current += consumer->drawnCurrent();
        }
    }

    // ── Step 3: push total load to active battery ─────────────────────
    // Battery forwards this so the model can update voltage
    m_batteries[m_activeBatteryIndex]->receiveLoad(totalCurrent, dt);


    // // ──  sum generator supply ──────────────────────────────────
    // float gen_supply_W = 0.0f;
    // for (auto* generator : m_generators) {
    //     gen_supply_W += generator->availablePowerW();
    // }

    // // ──  generator covers load or battery supplements ──────────
    // if (!m_generators.empty()){
    //     if(gen_supply_W >= total_demand_W){
    //         // generators cover everything - push load to all generators
    //         for (auto* generator : m_generators){
    //             generator->receiveLoad(total_current, dt);
    //         }
    //         // surplus -> charge target battery
    //         const float surplus_W = gen_supply_W - total_demand_W;
    //         const float surplus_current = (bus_voltage > 1e-6f)
    //                                        ? surplus_W / bus_voltage
    //                                        : 0.0f;
    //         PowerProvider* target = selectChargeTarget();
    //         if(target && surplus_current > 1e-6f){
    //             target->receiveCharge(surplus_current, dt);
    //         }
    //     } else {
    //         // batteries supplement the shortfall
    //         for(auto* generator : m_generators){
    //             generator->receiveLoad(total_current, dt);
    //         }
    //         const float shortfall_W = total_demand_W - gen_supply_W;
    //         const float shortfall_current = (bus_voltage > 1e-6f)
    //                                        ? shortfall_W / bus_voltage
    //                                        : total_current;

    //         if (!m_batteries.empty()){
    //             m_batteries[m_activeBatteryIndex]->receiveLoad(shortfall_current, dt);
    //         }
    //     } 
    // } else {
    //     // battery only -> active battery covers full demand
    //     if(!m_batteries.empty()){
    //         m_batteries[m_activeBatteryIndex]->receiveLoad(total_current, dt);
    //     }
    // }

    // ── Step 4: check active battery PowerLevel and act ───────────────
    const PowerLevel level = m_batteries[m_activeBatteryIndex]->powerLevel();
 
    if (level == PowerLevel::DEPLETED) {
        m_logger->info("[PowerManager] Vessel {} : battery {} {} is depleted", 
                        m_vesselName, m_activeBatteryIndex,
                        m_batteries[m_activeBatteryIndex]->name());
 
        if (!updateActiveProvider()) {
            // All batteries depleted -> cut power to all consumers
            m_logger->info("[PowerManager] Vessel {} : all baterries depleted -> cutting power.",
                            m_vesselName);
            for (auto& consumer : m_consumers) {
                consumer->deactivate();
            }
            return;
        }
        // Switched to new battery. log info and continue
        m_logger->info("[PowerManager] Vessel {} : switched to new battery {} {}", 
                        m_vesselName, m_activeBatteryIndex,
                        m_batteries[m_activeBatteryIndex]->name());
    }
 

    // ── Step 5: shed loads based on PowerLevel ───────
    shedLoadsIfNeeded(level);

    // ── Step 6: push updated voltage to all active consumers ──────────
    const float newVoltage =
        m_batteries[m_activeBatteryIndex]->voltage();
 
    for (auto& consumer : m_consumers) {
        consumer->receiveVoltage(newVoltage);
        consumer->update(_ecm);
    }
}

// ============================================================
// Private helpers
// ============================================================

void PowerManager::parsePowerProviders(const sdf::ElementPtr _sdf){
    // go through all <provider> tags in SDF
    // declaration order = priority order
    auto el = _sdf->GetElement("provider");
    while (el) {
        // type and name are mandatory 
        if (!el->HasAttribute("type")) {
            m_logger->info("PowerManager: <provider> tag missing required 'type' attribute -> skipping");
            el = el->GetNextElement("provider");
            continue;
        }
        if (!el->HasAttribute("name")) {
            m_logger->info("PowerManager: <provider> tag missing required 'name' attribute -> skipping");
            el = el->GetNextElement("provider");
            continue;
        }

        const std::string type = el->Get<std::string>("type");
        const std::string name = el->Get<std::string>("name");

        // add a new else-if branch when a new provider
        // type is introduced. IF MORE -> might need a "factory"
        if (type == "nuclear_battery") {
            m_providers.push_back(
                std::make_unique<NuclearBattery>(name, el, m_node));
        } else if (type == "diesel_generator") {
            m_providers.push_back(
                std::make_unique<DieselGenerator>(name, el, m_node));
        } else {
            m_logger->info("PowerManager: unknown provider type: {} -> skipping", type);
            el = el->GetNextElement("provider");
            continue;
        }
        // populate
        PowerProvider* prov = m_providers.back().get();
        if(prov->canReceiveCharge()){
            m_batteries.push_back(prov);
            m_logger->info("PowerManager: registered battery {} with type {}", name, type);
        }
        // else {
        //     m_generators.push_back(prov);
        //     m_logger->info("PowerManager: registered generator {} with type {}", name, type);
        // }
        el = el->GetNextElement("provider");
    }

    if(m_providers.empty()){
        m_logger->info("PowerManager: Vessel {} : no providers found in SDF", m_vessel_name);
    }
}

void PowerManager::parsePowerConsumers(
    const sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Sensors ───────────────────────────────────────────────────────
    // go through <sensor> tags that includes a power_type attribute
    // Priority level: 
    // 1 = safety critical         - never shed   e.g.: navigation, emergency light
    // 2 = operationally important - only shed for last resort
    // 3 = mission systems
    // 4 = non-essential           - shed first
    auto el = _sdf->GetElement("sensor");
    while(el){
        if(el->HasAttribute("power_type")){
            if (!el->HasAttribute("name")) {
                m_logger->info("[PowerManager] <sensor> with power_type {} missing a 'name' - skipping", power_type);
                el = el->GetNextElement("sensor");
                continue;
            }
            const std::string name = el->Get<std::string>("name");
            const float nominalW = el->Get<float>("nominal_w", 1.0f);
            const int priority = el->Get<int>("priority", 3);

            m_logger->info("PowerManager registered sensor consumer {} with priority {}", name, priority);
            m_consumers.push_back(
                    std::make_unique<SensorConsumer>(name, nominalW, priority, el, m_node, _ecm));
        }
        el = el->GetNextElement("sensor");
    }
    ////// will need to adapt depending on how generator is declared in SDF
    ////// maybe could be a link: <link name="thruster_link" <thruster> .... </thruster> </link>
    // ThrusterConsumer subscribes to /vessel_N/cmd_rpm  ------------------> might need to update!
    auto el = _sdf->GetElement("thruster");
    while (el) { 
        if (!el->HasAttribute("name")) {
            m_logger->info("[PowerManager] <thruster> missing required 'name' -> skipping");
            el = el->GetNextElement("thruster");
            continue;
        }
        const std::string name = el->Get<std::string>("name");
        const float nominalW = el->Get<float>("nominal_w", 1.0f);
        const int priority = el->Get<int>("priority", 2);

        m_consumers.push_back(
                std::make_unique<ThrusterConsumer>(name, nominalW, priority, el, m_node, _ecm));
        m_logger->info("PowerManager registered thruster consumer {} with priority {}", consumer, priority);

        el = el->GetNextElement("thruster");
    }
}

bool PowerManager::updateActiveProvider(){
    // find the next non-depleted provider
    // preserves the priority order
    for (int i = m_activeBatteryIndex; i < static_cast<int>(m_batteries.size()); ++i)
    {
        if(!m_providers[i]->isDepleted()){
            if(i != m_activeBatteryIndex){
                m_logger->info("PowerManager: Vessel {}: switching from provider {}
                     to provider {}", m_vessel_name, m_activeBatteryIndex, i);
                m_activeBatteryIndex = i;
            }
            return true;
        }
    }
    return false; // all providers are depleted, sad time :'(
}

// ignoring for now
// PowerProvider* PowerManager::selectChargeTarget() const
// {
//     if (m_batteries.empty()) {
//         return nullptr;
//     }
 
//     // Priority 1: first depleted battery —> restore it first
//     for (auto* battery : m_batteries) {
//         if (battery->isDepleted()) {
//             return battery;
//         }
//     }
 
//     // Priority 2: current active battery —> keep it topped up
//     return m_batteries[m_activeBatteryIndex];
// }

void PowerManager::shedLoadsIfNeeded(const PowerLevel level){
    // NORMAL -> no shedding needed
    if (level == PowerLevel::NORMAL || level == PowerLevel::DEPLETED) {
        return;
        // DEPLETED is handled before this call in Update(). if we reach
        // here with DEPLETED it means we just switched to a new battery
        // which starts at NORMAL, so no shedding needed.
    }
 
    if (level == PowerLevel::WARN) {
        // Level 1: voltage between 85–95% of voltage_min
        // Shed one non-essential (priority 4) consumer per tick
        m_logger->info("[PowerManager] Vessel {} : battery WARN level -> shedding non-essential loads",
                        m_vesselName);
 
        for (auto it = m_consumers.rbegin(); it != m_consumers.rend(); ++it) {
            if ((*it)->isActive() && (*it)->priority() == 4) {
                (*it)->deactivate();
                m_logger->info("[PowerManager] shed: {} (priority 4)", (*it)->name());
                return; // one per tick
            }
        }
        // No priority-4 consumers left, log but do not escalate yet
        m_logger->info("[PowerManager] Vessel {} : no priority-4 consumers left to shed", m_vesselName);
        return;
    }
 
    if (level == PowerLevel::CRITICAL) {
        // Level 2: voltage within 5% of voltage_min
        // Shed priority 3 and below
        m_logger->info("[PowerManager] Vessel {} : battery CRITICAL level -> shedding mission loads", m_vesselName);
 
        for (int group = 3; group >= 2; --group) {
            for (auto it = m_consumers.rbegin();
                 it != m_consumers.rend(); ++it)
            {
                if ((*it)->isActive() && (*it)->priority() == group) {
                    (*it)->deactivate();
                    m_logger->info("[PowerManager] shed: {} with priotity {}", (*it)->name(), group);
                    return; // one per tick
                }
            }
        }
 
        // Only priority 1 consumers remain 
        m_logger->info("[PowerManager] Vessel {} : critical, only safety-critical consumers remain", m_vesselName);
    }
}

bool PowerManager::matchesEntity(
    const std::string& consumerName,
    const gz::sim::Entity& _entity,
    gz::sim::EntityComponentManager& _ecm) const
{
    const auto* nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    return nameComp && nameComp->Data() == consumerName;
}

} //namespace lotusim::gazebo