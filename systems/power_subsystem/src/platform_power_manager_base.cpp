/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/platform_power_manager_base.hpp"

#include <gz/common/Console.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>

// abstract classes
#include "power_subsystem/power_consumer/power_consumer.hpp"
#include "power_subsystem/power_provider/power_provider.hpp"

// PowerProvider subclasses
#include "power_subsystem/power_provider/battery.hpp"
#include "power_subsystem/power_provider/generator.hpp"
#include "power_subsystem/power_provider/rpm_generator.hpp"
#include "power_subsystem/power_provider/simple_battery.hpp"
#include "power_subsystem/power_provider/simple_generator.hpp"

// PowerConsumer subclasses
#include "lotusim_common/common.hpp"
#include "power_subsystem/power_consumer/sensor_power_consumer.hpp"
#include "power_subsystem/power_consumer/thruster_power_consumer.hpp"

namespace lotusim::gazebo {

PlatformPowerManagerBase::PlatformPowerManager(
    gz::sim::Entity m_vessel_entity,
    const std::string& m_vessel_name,
    rclcpp::Node::SharedPtr m_node,
    sdf::ElementPtr sdfptr,
    gz::sim::EntityComponentManager& _ecm)
    : m_vessel_entity(m_vessel_entity)
    , m_vessel_name(m_vessel_name)
    , m_node(std::move(m_node))
{
    const std::string loggerName = m_vessel_name + "_power";
    m_logger =
        logger::createConsoleAndFileLogger(loggerName, loggerName + ".txt");

    if (!initPlatformPowerSystems(sdfptr)) {
        m_logger->error(
            "PlatformPowerManager [{}]: failed to initialize platfrom power systems",
            m_vessel_name);
        return;
    }

    m_power_status_pub =
        m_node->create_publisher<lotusim_msgs::msg::PowerStatus>(
            "/lotusim/" + m_vessel_name + "/power_status",
            rclcpp::QoS(rclcpp::KeepLast(10)));

    m_power_status_timer = m_node->create_wall_timer(
        std::chrono::seconds(2),
        [this]() { publishPowerStatus(); });

    sdf::ElementPtr rootEl = sdfptr->GetIncludeElement();
    if (!rootEl) {
        sdfptr = rootEl;
    }

    initPowerProvider(sdfptr);
    initPowerConsumers(sdfptr);

    m_logger->info(
        "PlatformPowerManagerBase::PlatformPowerManager:  [{}] initialization complete.",
        m_vessel_name);
}

PlatformPowerManagerBase::~PlatformPowerManager()
{
    consumer->eachDelete();
}

void PlatformPowerManagerBase::PostUpdate(
    const gz::sim::UpdateInfo& /*_info*/,
    const gz::sim::EntityComponentManager& _ecm)
{
    return;
}

void PlatformPowerManagerBase::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Guard ─────────────────────────────────────────────────────────
    // I dont think this is needed
    // if (!m_consumers_parsed) {
    //     return;
    // }

    if (m_batteries.empty()) {
        return;
    }

    const float dt =
        static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    handlePowerUpdate(dt, _ecm, _info);
}

float PlatformPowerManagerBase::activeBusVoltage() const
{
    if (!m_all_batteries_depleted)
        return m_batteries[m_active_battery_index]->voltage();

    for (auto& g : m_generators)
        if (!g->isDepleted())
            return g->voltage();

    return 1.0f;  // no source available
}

bool PlatformPowerManagerBase::initPowerProvider(sdf::ElementPtr sdfptr)
{
    if (!sdfptr) {
        m_logger->error(
            "PlatformPowerManagerBase::initPlatformPowerSystems [{}]: ModelSdf has no element",
            m_vessel_name);
        return false;
    }

    if (sdfptr->HasElement("lotusim_power")) {
        auto powerEl = sdfptr->GetElement("lotusim_power");

        while (powerEl) {
            if (!powerEl->HasElement("name")) {
                m_logger->error(
                    "PlatformPowerManager [{}]: <lotusim_power> is missing required <name> -> skipping",
                    m_vessel_name);
                powerEl = powerEl->GetNextElement("lotusim_power");
                continue;
            }
            const std::string providerName = powerEl->Get<std::string>("name");
            auto [provider, type] = PowerProvider::createFromSdf(
                providerName,
                powerEl,
                m_node,
                m_vessel_name,
                m_logger);
            if (provider) {
                if (type == ProviderType::Battery)
                    m_batteries.push_back(
                        std::static_pointer_cast<Battery>(provider));
                else
                    m_generators.push_back(
                        std::static_pointer_cast<Generator>(provider));
                m_providers.push_back(provider);
            }
            powerEl = powerEl->GetNextElement("lotusim_power");
        }
    }
    if (m_providers.empty()) {
        m_logger->warn(
            "PlatformPowerManager [{}]: no power providers found",
            m_vessel_name);
    }

    m_logger->info(
        "PlatformPowerManager [{}]: [{} batteries], [{} generators]",
        m_vessel_name,
        m_batteries.size(),
        m_generators.size());

    return true;
}

bool PlatformPowerManagerBase::initPowerConsumers(
    sdf::ElementPtr sdfptr,
    gz::sim::EntityComponentManager& _ecm)
{
    m_consumers.clear();

    if (!sdfptr) {
        m_logger->debug(
            "PlatformPowerManagerBase::initPowerConsumers [{}]: ModelSdf has no element",
            m_vessel_name);
        return false;
    }

    // DFS over the model SDF tree.
    // When a <lotusim_power> node is found, dispatch to the appropriate
    // consumer factory using the parent element for context, then stop
    // descending that branch.
    std::function<void(sdf::ElementPtr, sdf::ElementPtr)> dfs =
        [&](sdf::ElementPtr el, sdf::ElementPtr parent) {
            if (!el)
                return;

            if (el->GetName() == "lotusim_power") {
                if (!parent)
                    return;
                const std::string consumerName =
                    parent->GetAttribute("name")
                        ? parent->GetAttribute("name")->GetAsString()
                        : "";
                m_consumers.push_back(
                    auto [provider, type] = PowerConsumer::createFromSdf(
                        consumerName,
                        el,
                        m_node,
                        m_vessel_name,
                        m_logger););
                return;  // do not recurse into <lotusim_power>
            }

            auto child = el->GetFirstElement();
            while (child) {
                dfs(child, el);
                child = child->GetNextElement();
            }
        }

    dfs(sdfptr, nullptr);

    if (!m_consumers.empty()) {
        m_logger->infog(
            "PlatformPowerManagerBase::initPowerConsumers: [{}] found {} power consumers",
            m_vessel_name,
            m_consumers.size());
    }
    return true;
}

bool PlatformPowerManagerBase::updateActiveProvider()
{
    // find the next non-depleted provider
    // preserves the priority order
    for (int i = m_active_battery_index;
         i < static_cast<int>(m_batteries.size());
         ++i) {
        if (!m_batteries[i]->isDepleted()) {
            if (i != m_active_battery_index) {
                m_logger->info(
                    "PowerManager: Vessel {}: switching from provider {}",
                    m_vessel_name,
                    m_active_battery_index);
                m_active_battery_index = i;
            }
            return true;
        }
    }
    return false;  // all providers are depleted, sad time :'(
}

void PlatformPowerManagerBase::publishPowerStatus()
{
    if (m_providers.size() == 0) {
        return;
    }

    lotusim_msgs::msg::PowerStatus msg;
    msg.vessel_name = m_vessel_name;
    msg.provider_count = static_cast<uint32_t>(m_providers.size());

    if (!m_all_batteries_depleted) {
        const auto& bat = m_batteries[m_active_battery_index];
        msg.active_provider_name = bat->name();
        msg.active_provider_type = "battery";
        msg.active_provider_soc = bat->getStateOfCharge();
        msg.active_provider_voltage = bat->voltage();
    } else {
        std::shared_ptr<Generator> activeGen;
        for (auto& g : m_generators)
            if (!g->isDepleted()) {
                activeGen = g;
                break;
            }

        if (activeGen) {
            msg.active_provider_name = activeGen->name();
            msg.active_provider_type = "generator";
            msg.active_provider_soc = activeGen->getStateOfCharge();
            msg.active_provider_voltage = activeGen->voltage();
        } else {
            msg.active_provider_name = "none";
            msg.active_provider_type = "none";
            msg.active_provider_soc = 0.0f;
            msg.active_provider_voltage = 0.0f;
        }
    }
    m_power_status_pub->publish(msg);
}

}  // namespace lotusim::gazebo