/*
 * Copyright (c) 2026 Naval Group
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
#include "power_subsystem/power_strategy.hpp"
#include "power_subsystem/default_power_strategy.hpp"

// PowerProvider subclasses
#include "power_subsystem/simple_battery.hpp"
#include "power_subsystem/battery.hpp"
#include "power_subsystem/generator.hpp"
#include "power_subsystem/simple_generator.hpp"
#include "power_subsystem/rpm_generator.hpp"

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

    m_strategy = std::make_unique<DefaultPowerStrategy>(); // default load distribution and shedding logic
 
    if (!parsePowerProviders(_ecm)) {
        m_logger->error(
            "PowerManagerInstance [{}]: failed to parse power providers",
            m_vessel_name);
        return;
    }
}

void PowerManagerInstance::PostUpdate(
    const gz::sim::UpdateInfo& /*_info*/,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (!m_consumers_parsed) {
        parsePowerConsumers(const_cast<gz::sim::EntityComponentManager&>(_ecm));
        // only mark as done once we actually found consumers
        if (!m_consumers.empty()) {
            m_consumers_parsed = true;
            m_logger->info(
                "PowerManagerInstance [{}]: {} provider(s) [{} battery/ies], [{} generator/s], {} consumer(s)",
                m_vessel_name, m_providers.size(), m_batteries.size(), m_generators.size(), m_consumers.size()
            );
        } else {
            m_logger->debug(
                "PowerManagerInstance [{}]: no consumers yet, will retry next tick",
                m_vessel_name);
        }
        return;
    }
}

void PowerManagerInstance::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Guard ─────────────────────────────────────────────────────────
    // wait until consumers and batteries are parsed

    if (!m_consumers_parsed) { return; }
    if (m_batteries.empty()) { return; }

    const float dt = static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    // ── Step 1: topology change callbacks ─────────────────────────────
    _ecm.EachNew<gz::sim::components::CustomSensor,
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

    // use active battery voltage as bus voltage reference
    const float bus_voltage = m_batteries[m_activeBatteryIndex]->voltage();
    const float safe_voltage = (bus_voltage > 1e-6f) ? bus_voltage : 1.0f;
    //const float total_demand_W = total_current * safe_voltage;

    // Build the context passed to the strategy
    PowerStrategyContext context {
        .batteries            = m_batteries,
        .generators           = m_generators,
        .providers            = m_providers, 
        .consumers            = m_consumers,
        .activeBatteryIndex   = m_activeBatteryIndex,
        .allBatteriesDepleted = m_all_batteries_depleted,
        .totalCurrentA        = total_current,
        .busVoltage           = safe_voltage,
    };

    // ── Step 3: load distribution ─────────────────────────────────────
    //
    // Normal operation (batteries available):
    //   battery covers consumer demand
    //   generator charges the active battery (throttled to what it needs)
    //   generator does NOT run ahead of battery
    //
    // All batteries depleted (emergency):
    //   generator covers consumer demand directly
    //   reactivate consumers by priority up to generator capacity
    m_strategy->distributeLoad(context, dt, m_logger);

    // ── Step 4: check active battery PowerLevel and act ───────────────
    if (!m_all_batteries_depleted) {
        const PowerLevel level = m_batteries[m_activeBatteryIndex]->powerLevel();

        if (level == PowerLevel::DEPLETED) {
            m_logger->info("PowerManagerInstance [{}]: battery [{}] depleted",
                m_vessel_name, m_batteries[m_activeBatteryIndex]->name());

            m_strategy->handleDepleted(context, dt, m_logger);

            if (m_all_batteries_depleted) {
                Generator* activeGen = nullptr;
                for (auto* g : m_generators)
                    if (!g->isDepleted()) { activeGen = g; break; }
                context.busVoltage = activeBusVoltage();
                return;
            }
            // switched to new battery
            context.busVoltage = m_batteries[m_activeBatteryIndex]->voltage();
            if (context.busVoltage <= 1e-6f) context.busVoltage = activeBusVoltage();
        }

        // ── Step 5: shed loads ────────────────────────────────────────────
        m_strategy->shedLoads(context, level, m_logger);
    }

    // ── Step 6: push updated voltage to all active consumers ──────────
    Generator* activeGen = nullptr;
    for (auto* g : m_generators) {
        if (!g->isDepleted()) { 
            activeGen = g;
            break; 
        }
    }
    const float voltage = m_all_batteries_depleted
                        ? (activeGen ? activeGen->voltage() : 0.0f)
                        : m_batteries[m_activeBatteryIndex]->voltage();

    for (auto& consumer : m_consumers) {
        consumer->receiveVoltage(voltage);
        consumer->update(_ecm);
    }
}

// ============================================================
// Private helpers
// ============================================================

void PowerManagerInstance::parseOnePowerProvider(
    const std::string& providerName,
    const sdf::ElementPtr& powerSdf)
{
    if (!powerSdf->HasElement("type")) {
        m_logger->error(
            "PowerManagerInstance [{}]: provider [{}] has <lotusim_power> "
            "but missing required <type> -> skipping",
            m_vessel_name, providerName);
        return;
    }

    const std::string type = powerSdf->Get<std::string>("type");

    if (type == "simple_battery") {
        auto battery = std::make_unique<SimpleBattery>(providerName, powerSdf, m_node);
        m_batteries.push_back(battery.get());
        m_providers.push_back(std::move(battery));
    } else if (type == "simple_generator") {
        auto generator = std::make_unique<SimpleGenerator>(providerName, powerSdf, m_node);
        m_generators.push_back(generator.get());
        m_providers.push_back(std::move(generator));
    } else if (type == "rpm_generator") {
        auto generator = std::make_unique<RpmGenerator>(providerName, powerSdf, m_node, m_vessel_name);
        m_generators.push_back(generator.get());
        m_providers.push_back(std::move(generator));
    } else {
        m_logger->error(
            "PowerManagerInstance [{}]: unknown provider type '{}' for [{}] -> skipping",
            m_vessel_name, type, providerName);
    }
}

float PowerManagerInstance::activeBusVoltage() const
{
    if (!m_all_batteries_depleted)
        return m_batteries[m_activeBatteryIndex]->voltage();

    for (auto* g : m_generators)
        if (!g->isDepleted()) return g->voltage();

    return 1.0f; // no source available
}

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

    sdf::ElementPtr sdfptr = modelSdfComp->Data().Element();
    if (!sdfptr) {
        m_logger->error(
            "PowerManagerInstance [{}]: ModelSdf has no element",
            m_vessel_name);
        return false;
    }

    sdf::ElementPtr rootEl = sdfptr->GetIncludeElement();
    if (!rootEl) {
        rootEl = sdfptr;
    }

    if (rootEl->HasElement("lotusim_power")){
        auto powerEl = rootEl->GetElement("lotusim_power");
        
        while (powerEl) {
            if (!powerEl->HasElement("name")) {
                m_logger->error(
                    "PowerManagerInstance [{}]: <lotusim_power> is missing required <name> -> skipping",
                    m_vessel_name);
                powerEl = powerEl->GetNextElement("lotusim_power");
                continue;
            }
            const std::string providerName = powerEl->Get<std::string>("name");
            parseOnePowerProvider(providerName, powerEl);
            powerEl = powerEl->GetNextElement("lotusim_power");
        }
    }
    if (m_providers.empty()) {
        m_logger->warn(
            "PowerManagerInstance [{}]: no power providers found",
            m_vessel_name);
    }
    return true;
}

sdf::ElementPtr PowerManagerInstance::findRawSensorElement(
    const sdf::ElementPtr& modelEl,
    const std::string& sensorName)
{
    auto linkEl = modelEl->GetElement("link");
    while (linkEl) {
        auto sensorEl = linkEl->GetElement("sensor");
        while (sensorEl) {
            if (sensorEl->GetAttribute("name") &&
                sensorEl->GetAttribute("name")->GetAsString() == sensorName) {
                return sensorEl;  // raw element with all custom attributes intact
            }
            sensorEl = sensorEl->GetNextElement("sensor");
        }
        linkEl = linkEl->GetNextElement("link");
    }
    return nullptr;
}

bool PowerManagerInstance::parsePowerConsumers(
    gz::sim::EntityComponentManager& _ecm)
{
    // clear before re-parsing to avoid duplicates on retry
    m_consumers.clear();
    // get SDF
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
            "PowerManagerInstance [{}]: ModelSdf has no element",
            m_vessel_name);
        return false;
    }
    // ── Sensors ───────────────────────────────────────────────────────
    // Walk all CustomSensor entities in the ECM
    // For each one, walk up the parent chain to confirm it belongs to
    // this vessel. If it has a power_type attribute, register as consumer
    _ecm.Each<gz::sim::components::CustomSensor,
              gz::sim::components::ParentEntity,
              gz::sim::components::Name>(
        [&](const gz::sim::Entity& /*_entity*/,
            const gz::sim::components::CustomSensor* /*_custom*/,
            const gz::sim::components::ParentEntity* _parent,
            const gz::sim::components::Name* _name) -> bool
        {
            // walk up from sensor -> link -> model to confirm vessel ownership
            const auto* linkParent = _ecm.Component<gz::sim::components::ParentEntity>(_parent->Data());
            if (!linkParent || linkParent->Data() != m_vessel_entity) {
                return true; // not our vessel
            }
 
            // get power_type attribute from SDF
            const std::string name = _name->Data();
            const sdf::ElementPtr rawSensorEl = findRawSensorElement(modelEl, name);
            if (!rawSensorEl) {
                m_logger->warn(
                    "PowerManagerInstance [{}]: could not find raw SDF element for sensor [{}]",
                    m_vessel_name, name);
                return true;
            }

            if (!rawSensorEl->HasElement("lotusim_power")) {
                m_logger->warn(
                    "PowerManagerInstance [{}]: sensor [{}] does not have power_type -> skipping",
                    m_vessel_name, name);
                return true; // not a power-managed sensor, skip silently
            }

            const sdf::ElementPtr powerEl = rawSensorEl->GetElement("lotusim_power");
            const std::string powerType = powerEl->Get<std::string>("power_type", "").first;
            const float nominalW = powerEl->Get<float>("nominal_w", 1.0f).first;
            const int priority = powerEl->Get<int>("priority", 3).first;
 
            if (powerType.empty()) {
                m_logger->warn(
                    "PowerManagerInstance [{}]: sensor [{}] has <lotusim_power> but "
                    "missing <power_type> -> skipping",
                    m_vessel_name, name);
                return true;
            }
            // add else-if for new consumer type
            if (powerType == "sensor") {
                // m_consumers.push_back(std::make_unique<SensorPowerConsumer>(
                //     name, nominalW, priority, rawSensorEl, m_node, _ecm));

                auto consumer = std::make_unique<SensorPowerConsumer>(
                    name, nominalW, priority, rawSensorEl, m_node, _ecm);
 
                m_logger->info(
                    "PowerManagerInstance [{}]: registered sensor consumer [{}] nominal_w={} priority={}",
                    m_vessel_name, name, nominalW, priority);

                consumer->setServiceName(m_vessel_name); 
                m_consumers.push_back(std::move(consumer));
 
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
 
            m_consumers.push_back(std::make_unique<ThrusterPowerConsumer>(
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
        if(!m_batteries[i]->isDepleted()){
            if(i != m_activeBatteryIndex){
                m_logger->info("PowerManager: Vessel {}: switching from provider {}", m_vessel_name, m_activeBatteryIndex);
                m_activeBatteryIndex = i;
            }
            return true;
        }
    }
    return false; // all providers are depleted, sad time :'(
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