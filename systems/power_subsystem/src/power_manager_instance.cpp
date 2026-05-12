/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_manager_instance.hpp"

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/common/Console.hh>

// abstract classes
#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/power_consumer.hpp"

// PowerProvider subclasses
#include "power_subsystem/simple_battery.hpp"


// PowerConsumer subclasses
#include "power_subsystem/sensor_power_consumer.hpp"
#include "power_subsystem/thruster_power_consumer.hpp"

#include "lotusim_common/common.hpp" 

namespace lotusim::gazebo{

PowerManagerInstance::PowerManagerInstance(
    gz::sim::Entity m_vessel_entity,
    const std::string& m_vessel_name,
    rclcpp::Node::SharedPtr m_node,
    gz::sim::EntityComponentManager& _ecm)
    : m_vessel_entity(m_vessel_entity)
    , m_vessel_name(m_vessel_name)
    , m_node(std::move(m_node))
{
    const std::string loggerName = m_vessel_name + "_power";
    m_logger = logger::createConsoleAndFileLogger(
        loggerName, loggerName + ".txt");
 
    if (!parsePowerProviders(_ecm)) {
        m_logger->error(
            "PowerManagerInstance [{}]: failed to parse power providers",
            m_vessel_name);
        return;
    }

    if (!parsePowerConsumers(_ecm)) {
        m_logger->error(
            "PowerManagerInstance [{}]: failed to parse power consumers",
            m_vessel_name);
        return;
    }
 
    m_logger->info(
        "PowerManagerInstance [{}]: {} provider(s) [{} battery/ies], {} consumer(s)",
        m_vessel_name, m_providers.size(), m_batteries.size(), m_consumers.size()
    );
}


void PowerManagerInstance::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // dt in seconds —> needed by receiveLoad() for fuel/soc integration
    const float dt = static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    // ── Guard ─────────────────────────────────────────────────────────
    // if (m_providers.empty()){
    //     m_logger->info("PowerManagerInstance: Vessel {} has no power providers configured.", m_vessel_name);
    //     return;
    // }
    // replace once implemented!

    if (m_batteries.empty()) {
        return;
    }

    // ── Step 1: topology change callbacks ─────────────────────────────
    // EachNew fires when a CustomSensor component appears in the ECM
    // EachRemoved fires when one is removed
    // Thrusters: no component equivalent yet 
    _ecm.EachNew<gz::sim::components::CustomSensor,
                 gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::CustomSensor*,
            const gz::sim::components::ParentEntity* _parent) -> bool
        {
            // check if this sensor belongs to our vessel
            if (!isChildOfVessel(_entity, _parent->Data(), _ecm)) {
                return true;
            }
            for (auto& consumer : m_consumers) {
                if (matchesEntity(consumer->name(), _entity, _ecm)) {
                    consumer->eachNew();
                }
            }
            return true;
        }
    );
 
    _ecm.EachRemoved<gz::sim::components::CustomSensor,
                     gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::CustomSensor*,
            const gz::sim::components::ParentEntity* _parent) -> bool
        {
            if (!isChildOfVessel(_entity, _parent->Data(), _ecm)) {
                return true;
            }
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
    m_batteries[m_activeBatteryIndex]->receiveLoad(total_current, dt);


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
        m_logger->info("PowerManagerInstance [{}]: battery [{}] depleted", 
                        m_vessel_name,
                        m_batteries[m_activeBatteryIndex]->name());
 
        if (!updateActiveProvider()) {
            // All batteries depleted -> cut power to all consumers
            m_logger->warn("PowerManagerInstance [{}]: all batteries depleted cutting power",
                            m_vessel_name);
            for (auto& consumer : m_consumers) {
                consumer->deactivate();
            }
            return;
        }
        // Switched to new battery. log info and continue
        m_logger->info("PowerManagerInstance [{}]: switched to battery [{}]", 
                        m_vessel_name,
                        m_batteries[m_activeBatteryIndex]->name());
    }
 

    // ── Step 5: shed loads based on PowerLevel ───────
    shedLoadsIfNeeded(level);

    // ── Step 6: push updated voltage to all active consumers ──────────
    const float voltage = m_batteries[m_activeBatteryIndex]->voltage();
    for (auto& consumer : m_consumers) {
        consumer->receiveVoltage(voltage);
        consumer->update(_ecm);
    }
}

// ============================================================
// Private helpers
// ============================================================

bool PowerManagerInstance::parsePowerProviders(gz::sim::EntityComponentManager& _ecm)
{
    // Get the full model SDF from the ModelSdf component
    const auto* modelSdfComp = _ecm.Component<gz::sim::components::ModelSdf>(m_vessel_entity);
    if (!modelSdfComp) {
        m_logger->error(
            "PowerManagerInstance [{}]: no ModelSdf component found",
            m_vessel_name);
        return false;
    }

    // Walk <link> elements in the SDF
    const sdf::ElementPtr modelEl = modelSdfComp->Data().Element();
    if (!modelEl) {
        m_logger->error(
            "PowerManagerInstance [{}]: ModelSdf has no element",
            m_vessel_name);
        return false;
    }

    auto linkEl = modelEl->GetElement("link");
    while (linkEl) {
        // Check for <lotusim_power> tag
        if (!linkEl->HasElement("lotusim_power")) {
            linkEl = linkEl->GetNextElement("link");
            continue;
        }

        const std::string linkName = linkEl->Get<std::string>("name");
        const sdf::ElementPtr powerSdf = linkEl->GetElement("lotusim_power");

        if (!powerSdf->HasElement("type")) {
            m_logger->error(
                "PowerManagerInstance [{}]: link [{}] has <lotusim_power> but missing required <type> -> skipping",
                m_vessel_name, linkName);
                linkEl = linkEl->GetNextElement("link");
            continue;
        }

        const std::string type = powerSdf->Get<std::string>("type");

        if (type == "simple_battery") {
            m_providers.push_back(
                std::make_unique<SimpleBattery>(linkName, powerSdf, m_node));
        } else {
            m_logger->error(
                "PowerManagerInstance [{}]: unknown provider type '{}' on link [{}] -> skipping",
                m_vessel_name, type, linkName);
            linkEl = linkEl->GetNextElement("link");
            continue;
        }
        // TODO later
        // else {
        //     m_generators.push_back(raw);
        // }

        PowerProvider* raw = m_providers.back().get();
        if (raw->canReceiveCharge()) {
            m_batteries.push_back(raw);
            m_logger->info(
                "PowerManagerInstance [{}]: registered battery [{}] ({})",
                m_vessel_name, linkName, type);
        }

        linkEl = linkEl->GetNextElement("link");
    }

    if (m_providers.empty()) {
        m_logger->warn(
            "PowerManagerInstance [{}]: no power providers found",
            m_vessel_name);
    } 
    return true;    
}

bool PowerManagerInstance::parsePowerConsumers(
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Sensors ───────────────────────────────────────────────────────
    // Walk all CustomSensor entities in the ECM
    // For each one, walk up the parent chain to confirm it belongs to
    // this vessel. If it has a power_type attribute, register as consumer
    // Priority level: 
    // 1 = safety critical         - never shed   e.g.: navigation, emergency light
    // 2 = operationally important - only shed for last resort
    // 3 = mission systems
    // 4 = non-essential           - shed first
    _ecm.Each<gz::sim::components::CustomSensor,
              gz::sim::components::ParentEntity,
              gz::sim::components::Name>(
        [&](const gz::sim::Entity& /*_entity*/,
            const gz::sim::components::CustomSensor* _custom,
            const gz::sim::components::ParentEntity* _parent,
            const gz::sim::components::Name* _name) -> bool
        {
            // walk up from sensor -> link -> model to confirm vessel ownership
            const auto* linkParent = _ecm.Component<gz::sim::components::ParentEntity>(_parent->Data());
            if (!linkParent || linkParent->Data() != m_vessel_entity) {
                return true; // not our vessel
            }
 
            // get power_type attribute from SDF
            const sdf::Sensor& sdfSensor = _custom->Data();
            const sdf::ElementPtr sdfEl  = sdfSensor.Element();
 
            if (!sdfEl || !sdfEl->HasAttribute("power_type")) {
                return true; // no power config, skip silently
            }
 
            const std::string powerType = sdfEl->Get<std::string>("power_type");
            const std::string name = _name->Data();
            const float nominalW = sdfEl->Get<float>("nominal_w", 1.0f).first;
            const int priority = sdfEl->Get<int>("priority", 3).first;
 
            // construction 
            // add else-if for new consumer type
            if (powerType == "sensor") {
                m_consumers.push_back(std::make_unique<SensorPowerConsumer>(
                    name, nominalW, priority, sdfEl, m_node, _ecm));
 
                m_logger->info(
                    "PowerManagerInstance [{}]: registered sensor consumer [{}] nominal_w={} priority={}",
                    m_vessel_name, name, nominalW, priority);
 
            } else {
                m_logger->warn(
                    "PowerManagerInstance [{}]: unknown power_type '{}' on sensor [{}] -> skipping",
                    m_vessel_name, powerType, name);
            }
 
            return true;
        });


    // ── Thrusters ─────────────────────────────────────────────────────
    // walk all links belonging to this vessel looking for <thruster> tags
    // that carry power config
    // No dedicated thruster ECM component exists yet 
    const auto* modelSdfComp = _ecm.Component<gz::sim::components::ModelSdf>(m_vessel_entity);
    if (!modelSdfComp) {
        m_logger->error(
            "PowerManagerInstance [{}]: no ModelSdf component found",
            m_vessel_name);
        return false;
    }
    const sdf::ElementPtr modelEl = modelSdfComp->Data().Element();
    if (!modelEl) {
        m_logger->error(
            "PowerManagerInstance [{}]: ModelSdf has no element, cannot parse thrusters",
            m_vessel_name);
        return false;
    }

    // Walk all <link> elements in the model SDF
    auto linkEl = modelEl->GetElement("link");
    while (linkEl) {
        // Walk <thruster> child elements of this link
        auto thrusterEl = linkEl->GetElement("thruster");
        while (thrusterEl) {
            if (!thrusterEl->HasAttribute("name")) {
                m_logger->error(
                    "PowerManagerInstance [{}]: <thruster> missing "
                    "required 'name' attribute -> skipping",
                    m_vessel_name);
                thrusterEl = thrusterEl->GetNextElement("thruster");
                continue;
            }
 
            const std::string thrusterName = thrusterEl->Get<std::string>("name");
            const float nominalW = thrusterEl->Get<float>("nominal_w", 1.0f).first;
            const int priority = thrusterEl->Get<int>("priority", 2).first;
 
            m_consumers.push_back(
                std::make_unique<ThrusterPowerConsumer>(
                    thrusterName, nominalW, priority,
                    thrusterEl, m_node, _ecm));
 
            m_logger->info(
                "PowerManagerInstance [{}]: registered thruster consumer "
                "[{}] nominal_w={} priority={}",
                m_vessel_name, thrusterName, nominalW, priority);
 
            thrusterEl = thrusterEl->GetNextElement("thruster");
        }
 
        linkEl = linkEl->GetNextElement("link");
    }
 
    if (m_consumers.empty()) {
        m_logger->warn(
            "PowerManagerInstance [{}]: no power consumers found",
            m_vessel_name);
    }
 
    return true;
}

bool PowerManagerInstance::updateActiveProvider(){
    // find the next non-depleted provider
    // preserves the priority order
    for (int i = m_activeBatteryIndex; i < static_cast<int>(m_batteries.size()); ++i)
    {
        if(!m_providers[i]->isDepleted()){
            if(i != m_activeBatteryIndex){
                m_logger->info("PowerManager: Vessel {}: switching from provider {}", m_vessel_name, m_activeBatteryIndex);
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

void PowerManagerInstance::shedLoadsIfNeeded(const PowerLevel level){
    // NORMAL -> no shedding needed
    if (level == PowerLevel::NORMAL || level == PowerLevel::DEPLETED) {
        return;
        // DEPLETED is handled before this call in Update(). if we reach
        // here with DEPLETED it means we just switched to a new battery
        // which starts at NORMAL, so no shedding needed.
    }

    const int maxGroup = (level == PowerLevel::WARN) ? 4 : 3;
 
    for (int group = maxGroup; group >= 2; --group) {
        for (auto it = m_consumers.rbegin();
             it != m_consumers.rend(); ++it)
        {
            if ((*it)->isActive() && (*it)->priority() == group) {
                (*it)->deactivate();
                m_logger->warn(
                    "PowerManagerInstance [{}]: shed consumer [{}] "
                    "(priority {})",
                    m_vessel_name, (*it)->name(), group);
                return; // one per tick
            }
        }
    }
 
    // Only priority 1 consumers remain 
    m_logger->warn("PowerManagerInstance [{}]: critical -> only safety-critical "
        "consumers remain", m_vessel_name);
    
}

bool PowerManagerInstance::matchesEntity(
    const std::string& consumerName,
    const gz::sim::Entity& _entity,
    gz::sim::EntityComponentManager& _ecm) const
{
    const auto* nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    return nameComp && nameComp->Data() == consumerName;
}

bool PowerManagerInstance::isChildOfVessel(
    const gz::sim::Entity& /*_entity*/,
    const gz::sim::Entity& parentEntity,
    gz::sim::EntityComponentManager& _ecm) const
{
    // check that link's parent is our vessel model
    const auto* linkParent = _ecm.Component<gz::sim::components::ParentEntity>(parentEntity);
    return linkParent && linkParent->Data() == m_vessel_entity;
}

} //namespace lotusim::gazebo