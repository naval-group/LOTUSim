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

PlatformPowerManagerBase::PlatformPowerManagerBase(
    const gz::sim::Entity& vessel_entity,
    const std::string& vessel_name,
    rclcpp::Node::SharedPtr node,
    sdf::ElementPtr sdfptr)
    : m_vessel_entity(vessel_entity)
    , m_vessel_name(vessel_name)
    , m_node(std::move(node))
{
    const std::string loggerName = m_vessel_name + "_power";
    m_logger =
        logger::createConsoleAndFileLogger(loggerName, loggerName + ".txt");

    sdf::ElementPtr rootEl = sdfptr->GetIncludeElement();
    if (rootEl) {
        sdfptr = rootEl;
    }

    if (!initPowerProvider(sdfptr)) {
        m_logger->error(
            "PlatformPowerManagerBase [{}]: failed to initialize platfrom power provider",
            m_vessel_name);
    }

    if (!initPowerConsumers(sdfptr)) {
        m_logger->error(
            "PlatformPowerManagerBase [{}]: failed to initialize platfrom power consumer",
            m_vessel_name);
    }

    if (!updateActiveProvider()) {
        m_logger->warn(
            "PlatformPowerManagerBase [{}]: no non-depleted battery at startup",
            m_vessel_name);
    }

    m_power_status_pub =
        m_node->create_publisher<lotusim_msgs::msg::PowerStatus>(
            "/lotusim/" + m_vessel_name + "/power_status",
            rclcpp::QoS(rclcpp::KeepLast(10)));

    m_power_status_timer = m_node->create_wall_timer(
        std::chrono::seconds(2),
        [this]() { publishPowerStatus(); });

    m_logger->info(
        "PlatformPowerManagerBase::PlatformPowerManagerBase:  [{}] initialization complete.",
        m_vessel_name);
}

PlatformPowerManagerBase::~PlatformPowerManagerBase() = default;

void PlatformPowerManagerBase::PostUpdate(float dt)
{
    updateActiveProvider();
    return;
}

void PlatformPowerManagerBase::Update(float dt)
{
    // ── Guard ─────────────────────────────────────────────────────────
    // I dont think this is needed
    // if (!m_consumers_parsed) {
    //     return;
    // }

    if (m_batteries.empty() || m_active_battery_index < 0) {
        return;
    }

    handlePowerUpdate(dt);
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
                    "PlatformPowerManagerBase [{}]: <lotusim_power> is missing required <name> -> skipping",
                    m_vessel_name);
                powerEl = powerEl->GetNextElement("lotusim_power");
                continue;
            }
            const std::string providerName = powerEl->Get<std::string>("name");
            auto [provider, type] = PowerProvider::createFromSdf(
                providerName,
                m_vessel_name,
                powerEl,
                m_node,
                m_logger);
            if (provider) {
                if (provider->canReceiveCharge())
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
            "PlatformPowerManagerBase [{}]: no power providers found",
            m_vessel_name);
    }

    m_logger->info(
        "PlatformPowerManagerBase [{}]: [{} batteries], [{} generators]",
        m_vessel_name,
        m_batteries.size(),
        m_generators.size());

    return true;
}

bool PlatformPowerManagerBase::initPowerConsumers(sdf::ElementPtr sdfptr)
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
    std::function<void(sdf::ElementPtr, sdf::ElementPtr)> dfs = [&](sdf::
                                                                        ElementPtr
                                                                            el,
                                                                    sdf::ElementPtr
                                                                        parent) {
        if (!el)
            return;

        if (el->GetName() == "lotusim_power") {
            if (!parent)
                return;

            const std::string parentName = parent->GetName();
            if (parentName == "model")
                return;  // skip -> this is a provider definition

            const std::string consumerName =
                parent->GetAttribute("name")
                    ? parent->GetAttribute("name")->GetAsString()
                    : "";
            auto [consumer, type] = PowerConsumer::createFromSdf(
                consumerName,
                m_vessel_name,
                parent,
                m_node,
                m_logger);
            if (consumer) {
                m_consumers.push_back(consumer);
            } else {
                m_logger->warn(
                    "PlatformPowerManagerBase [{}]: failed to create consumer [{}], skipping",
                    m_vessel_name,
                    consumerName);
            }
            return;  // do not recurse into <lotusim_power>
        }

        auto child = el->GetFirstElement();
        while (child) {
            dfs(child, el);
            child = child->GetNextElement();
        }
    };

    dfs(sdfptr, nullptr);

    if (!m_consumers.empty()) {
        m_logger->info(
            "PlatformPowerManagerBase::initPowerConsumers: [{}] found {} power consumers",
            m_vessel_name,
            m_consumers.size());
    }
    return true;
}

bool PlatformPowerManagerBase::updateActiveProvider()
{
    const int startIndex =
        (m_active_battery_index < 0)
            ? 0
            : m_active_battery_index;  // at the start when init at -1
    // find the next non-depleted provider
    // preserves the priority order
    for (int i = startIndex; i < static_cast<int>(m_batteries.size()); ++i) {
        if (!m_batteries[i]->isDepleted()) {
            if (i != m_active_battery_index) {
                if (m_active_battery_index >= 0) {
                    m_logger->info(
                        "PowerManager: Vessel {}: switching from provider {}",
                        m_vessel_name,
                        m_active_battery_index);
                }
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

    for (auto&& provider : m_providers) {
        msg.providers_name.push_back(provider->name());
        msg.providers_type.push_back(std::string(toString(provider->type())));
        msg.providers_soc.push_back(provider->getStateOfCharge());
        msg.providers_voltage.push_back(provider->voltage());
        msg.active_provider = m_batteries[m_active_battery_index]->name();
    }
    m_power_status_pub->publish(msg);
}

}  // namespace lotusim::gazebo