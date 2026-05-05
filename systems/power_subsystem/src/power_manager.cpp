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
#include <gz/common/Console.hh>

#include "power_subsystem/power_provider.hpp"
#include "power_subsystem/power_consumer.hpp"

// Concrete PowerProvider subclasses
#include "battery_provider_base.hpp"
#include "generator_provider_base.hpp"

// Concrete PowerConsumer subclasses
#include "sensor_consumer_base.hpp"
#include "thruster_consumer_base.hpp"

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
    gz::sim::EventManager&)
{
    m_entity = _entity;

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

    // ── soc_min threshold ─────────────────────────────────────────────
    // parse sdf
    // m_socMinThreshold is a vessel-level safety margin
    // Example: 0.1 means shed when available power < demand + 10% headroom
    if(_sdf->HasElement("battery")){
        const auto batteryE1 = _sdf->GetElement("battery");
        m_socMinThreshold = batteryE1->Get<float>("soc_min", 0.1f);
    }

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


    // ── Step 1: sum consumer demand ───────────────────────────────────
    //each consumer calculates its own individual draw
    // PowerManager owns the sum
    float total_current = 0.0f;
    for(const auto& consumer : m_consumers){
        if(consumer->isActive()){
            total_current += consumer->drawnCurrent();
        }
    }

    // Use active battery voltage as bus voltage reference
    // Falls back to 0 if no batteries configured 
    const float bus_voltage = m_batteries.empty()
        ? 0.0f
        : m_batteries[m_activeBatteryIndex]->voltage();

    const float total_demand_W = total_current * bus_voltage;


    // ── Step 2: sum generator supply ──────────────────────────────────
    float gen_supply_W = 0.0f;
    for (auto* generator : m_generators) {
        gen_supply_W += generator->availablePowerW();
    }


    // ── Step 3: generator covers load or battery supplements ──────────
    if (!m_generators.empty()){
        if(gen_supply_W >= total_demand_W){
            // generators cover everything - push load to all generators
            for (auto* generator : m_generators){
                generator->receiveLoad(total_current, dt);
            }
            // surplus -> charge target battery
            const float surplus_W = gen_supply_W - total_demand_W;
            const float surplus_current = (bus_voltage > 1e-6f)
                                           ? surplus_W / bus_voltage
                                           : 0.0f;
            PowerProvider* target = selectChargeTarget();
            if(target && surplus_current > 1e-6f){
                target->receiveCharge(surplus_current, dt);
            }
        } else {
            // batteries supplement the shortfall
            for(auto* generator : m_generators){
                generator->receiveLoad(total_current, dt);
            }
            const float shortfall_W = total_demand_W - gen_supply_W;
            const float shortfall_current = (bus_voltage > 1e-6f)
                                           ? shortfall_W / bus_voltage
                                           : total_current;

            if (!m_batteries.empty()){
                m_batteries[m_activeBatteryIndex]->receiveLoad(shortfall_current, dt);
            }
        } 
    } else {
        // battery only -> active battery covers full demand
        if(!m_batteries.empty()){
            m_batteries[m_activeBatteryIndex]->receiveLoad(total_current, dt);
        }
    }

    // ── Step 4: switch battery if soc dropped below threshold ─────────
    if(!m_batteries.empty()){
        const float soc = m_batteries[m_activeBatteryIndex]->getStateOfCharge();

        if(soc < m_socMinThreshold){
            if(!updateActiveProvider){
                // all batteries depleted -> cut power to everything
                m_logger->info("PowerManager: Vessel {} has all power providers depleted - cutting power", 
                            m_vessel_name);
                for(auto& consumer : m_consumers){
                    consumer->deactivate();
                }
                return;
            }
        }
    }

    // ── Step 5: shed non-essential loads if power margin is low ───────
    shedLoadsIfNeeded();

    // ── Step 6: push updated voltage to all active consumers ──────────
    // re-read voltage after provider may have switched in step 4
    const float newVoltage = m_batteries.empty()
        ? (m_generators.empty()  ? 0.0f
                                 : m_generators[0]->voltage())
        : m_batteries[m_activeBatteryIndex]->voltage();
 
    for (auto& consumer : m_consumers) {
        consumer->receiveVoltage(newVoltage);
        consumer->update();
    }
}

// ============================================================
// Private helpers
// ============================================================

void PowerManager::parsePowerProviders(const sdf::ElementPtr _sdf){
    // go through all <battery> tags in SDF
    // declaration order = priority order
    auto el = _sdf->GetElement("provider");  ///////////// MIGHT NEED TO SWITCH TO PROVIDER TO INCLUDE GENERATOR
    while (el) {
        // type and name are mandatory 
        if (!el->HasAttribute("type")) {
            m_logger->info("PowerManager: <provider> tag missing required 'type' attribute — skipping");
            el = el->GetNextElement("battery");
            continue;
        }
        if (!el->HasAttribute("name")) {
            m_logger->info("PowerManager: <provider> tag missing required 'name' attribute — skipping");
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
            m_logger->info("PowerManager: unknown provider type: {} - skipping", type);
            el = el->GetNextElement("provider");
            continue;
        }
        // populate
        // m_batteries and m_generators point into m_providers
        PowerProvider* prov = m_providers.back().get();
        if(prov->canReceiveCharge()){
            m_batteries.push_back(prov);
            m_logger->info("PowerManager: registered battery {} with type {}", name, type);
        }
        else {
            m_generators.push_back(prov);
            m_logger->info("PowerManager: registered generator {} with type {}", name, type);
        }
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
    // go through <sensor> tags that includes a power_type attribute
    // <sensor> without this attribute are ignored by the power subsystem
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
            const std::string power_type = el->Get<std::string>("power_type");
            const std::string power_name = el->Get<std::string>("power_name");
            const float nominalW = el->Get<float>("nominal_w", 1.0f);
            const int priority = el->Get<int>("priority", 3);

            if (powerType == "sensor") {
                m_logger->info("PowerManager registered consumer {} : type {} and priority {}", consumer, power_type, priority);
                m_consumers.push_back(
                    std::make_unique<SensorConsumer>(power_name, nominalW, priority, el, m_node, _ecm));
            } else {
                m_logger->info("PowerManager: unknown consumer type: {} for consumer {} -> skipping", power_type, consumer);
                el = el->GetNextElement("sensor");
                continue;
            }
        }
        el = el->GetNextElement("sensor");
    }
    ////// will need to adapt depending on how generator is declared in SDF
    ////// maybe could be a link: <link name="thruster_link" <thruster> .... </thruster> </link>
    auto el = _sdf->GetElement("thruster");
    while (el) { 
        if(el->HasAttribute("name")){
            const std::string power_name = el->Get<std::string>("power_name");
            const float nominalW = el->Get<float>("nominal_w", 1.0f);
            const int priority = el->Get<int>("priority", 2);

            m_logger->info("PowerManager registered consumer {} : type {} and priority {}", consumer, power_type, priority);
            m_consumers.push_back(
                    std::make_unique<ThrusterConsumer>(power_name, nominalW, priority, el, m_node, _ecm));
        } else {
                m_logger->info("PowerManager: unknown consumer type: {} for consumer {} -> skipping", power_type, consumer);
                el = el->GetNextElement("thruster");
                continue;
        }
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

PowerProvider* PowerManager::selectChargeTarget() const
{
    if (m_batteries.empty()) {
        return nullptr;
    }
 
    // Priority 1: first depleted battery —> restore it first
    for (auto* battery : m_batteries) {
        if (battery->isDepleted()) {
            return battery;
        }
    }
 
    // Priority 2: current active battery —> keep it topped up
    return m_batteries[m_activeBatteryIndex];
}

void PowerManager::shedLoadsIfNeeded(){

    // ── Available power ───────────────────────────────────────────────
    // Sum availablePowerW() across all non-depleted providers
    // each provider implements this for its own type
    float available_power = 0.0f;
    for(const auto& provider : m_providers){
        if(!provider->isDepleted()){
            available_power += provider->availablePowerW();
        }
    }

    // ── Total demand ──────────────────────────────────────────────────
    float total_demand = 0.0f;
    for (const auto& consumer : m_consumers){
        if(consumer->isActive()){
            total_demand += consumer->nominalPowerW();
        }
    }

    // ── Margin check ──────────────────────────────────────────────────
    // shedding fires when available_power < total_demand * (1 + threshold)
    if(available_power >= total_demand * (1.0f + m_socMinThreshold)){
        return;
    }

    m_logger->info("PowerManager: Vessel {}: low power margin, available now: {}Wh with demand: 
        {}W -> shedding.", m_vessel_name, available_power, total_demand);

    // ── Priority-based shedding ───────────────────────────────────────
    // re-evaluated every sim tick
    // TO DO --------------------> add reactivation logic - when available power
    // recovers above threshold
    for (int group 4; group >=2; --group){
        for (auto it = m_consumers.rbegin(); it !=m_consumers.rend(); ++it){
            if ((*it)->isActive() && (*it)->priority() == group){
                (*it)->deactivate();
                m_logger->info("PowerManager: vessel {} shed consumer: {}", m_vessel_name, (*it)->name());
                return;
            }
        }
    }
    m_logger->info("PowerManager: Vessel {}: critical - only safety-critical consumers remain active", m_vessel_name);

}

} //namespace lotusim::gazebo