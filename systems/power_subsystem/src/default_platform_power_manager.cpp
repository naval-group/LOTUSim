/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/default_platform_power_manager.hpp"

#include <algorithm>

namespace lotusim::gazebo {

DefaultPlatformPowerManager::DefaultPlatformPowerManager(
    const gz::sim::Entity& vessel_entity,
    const std::string& vessel_name,
    rclcpp::Node::SharedPtr node,
    sdf::ElementPtr sdfptr)
    : PlatformPowerManagerBase(vessel_entity, vessel_name, node, sdfptr)
{
}

std::shared_ptr<Generator> DefaultPlatformPowerManager::firstActiveGenerator()
{
    for (auto& g : m_generators)
        if (!g->isDepleted())
            return g;
    return nullptr;
}

void DefaultPlatformPowerManager::handlePowerUpdate(float dt)
{
    // Step 1: sum consumer demand
    float total_current = 0.0f;
    for (const auto& consumer : m_consumers) {
        if (consumer->isActive())
            total_current += consumer->drawnCurrent();
    }

    // Step 2: bus voltage reference
    float bus_voltage = activeBusVoltage();

    // Step 3: distribute load across sources
    distributeLoad(dt, total_current, bus_voltage);

    // Step 4: check active battery level and act
    if (!m_all_batteries_depleted) {
        const PowerLevel level =
            m_batteries[m_active_battery_index]->powerLevel();

        if (level == PowerLevel::DEPLETED) {
            m_logger->info(
                "PlatformPowerManager [{}]: battery [{}] depleted",
                m_vessel_name,
                m_batteries[m_active_battery_index]->name());

            if (!handleDepleted(dt, bus_voltage)) {
                // all batteries depleted — push final voltage and return
                for (auto& consumer : m_consumers) {
                    consumer->receiveVoltage(bus_voltage);
                    consumer->update();
                }
                return;
            }
            // switched to new battery — update bus reference
            bus_voltage = m_batteries[m_active_battery_index]->voltage();
            if (bus_voltage <= 1e-6f)
                bus_voltage = activeBusVoltage();
        }

        // Step 5: shed loads based on power level
        shedLoads(level);
    }

    // Step 6: push updated voltage to all consumers
    std::shared_ptr<Generator> active_gen;
    for (auto& g : m_generators) {
        if (!g->isDepleted()) {
            active_gen = g;
            break;
        }
    }
    const float voltage = m_all_batteries_depleted
                              ? (active_gen ? active_gen->voltage() : 0.0f)
                              : m_batteries[m_active_battery_index]->voltage();

    for (auto& consumer : m_consumers) {
        consumer->receiveVoltage(voltage);
        consumer->update();
    }
}

void DefaultPlatformPowerManager::distributeLoad(
    float dt,
    float total_current_a,
    float bus_voltage)
{
    auto active_gen = firstActiveGenerator();

    if (!m_all_batteries_depleted) {
        m_batteries[m_active_battery_index]->receiveLoad(total_current_a, dt);

        int available_gen_count = 0;
        for (const auto& g : m_generators)
            if (!g->isDepleted())
                available_gen_count++;

        const bool can_charge = active_gen && (!m_all_batteries_depleted ||
                                               available_gen_count > 1);

        if (can_charge) {
            auto charge_target = m_batteries[m_active_battery_index];
            if (charge_target->getStateOfCharge() >= 1.0f) {
                charge_target = nullptr;
                for (auto& bat : m_batteries) {
                    if (bat->getStateOfCharge() < 1.0f) {
                        charge_target = bat;
                        break;
                    }
                }
            }
            if (charge_target) {
                if (charge_target->isDepleted()) {
                    m_logger->debug(
                        "DefaultPlatformPowerManager: charge target [{}] depleted "
                        "after load, skipping charge",
                        charge_target->name());
                } else {
                    const float charge_current = computeChargeCurrentA(
                        active_gen,
                        charge_target,
                        bus_voltage);
                    if (charge_current > 1e-6f) {
                        active_gen->receiveLoad(charge_current, dt);
                        charge_target->receiveLoad(charge_current, dt);
                    }
                }
            }
        }
    } else {
        // emergency: generator covers consumers directly
        if (active_gen) {
            const float gen_available_W = active_gen->availablePowerW();
            const float gen_current =
                std::min(total_current_a, gen_available_W / bus_voltage);
            active_gen->receiveLoad(gen_current, dt);

            m_logger->debug(
                "DefaultPlatformPowerManager: emergency – generator [{}] "
                "covering {:.1f} W of {:.1f} W demand",
                active_gen->name(),
                gen_current * bus_voltage,
                total_current_a * bus_voltage);

            const float headroom_W =
                gen_available_W - (gen_current * bus_voltage);
            reactivateIfPossible(headroom_W, bus_voltage);
        } else {
            m_logger->debug(
                "DefaultPlatformPowerManager: no power source available");
        }
    }
}

bool DefaultPlatformPowerManager::handleDepleted(
    float /*dt*/,
    float& bus_voltage)
{
    for (int i = m_active_battery_index;
         i < static_cast<int>(m_batteries.size());
         ++i) {
        if (!m_batteries[i]->isDepleted()) {
            if (i != m_active_battery_index) {
                m_logger->info(
                    "DefaultPlatformPowerManager: switching to battery [{}]",
                    m_batteries[i]->name());
                m_active_battery_index = i;
            }
            m_all_batteries_depleted = false;
            return true;
        }
    }

    // all batteries depleted
    m_all_batteries_depleted = true;
    m_logger->warn("DefaultPlatformPowerManager: all batteries depleted");

    auto active_gen = firstActiveGenerator();

    if (active_gen) {
        m_logger->info(
            "DefaultPlatformPowerManager: generator [{}] taking over consumer demand",
            active_gen->name());

        bus_voltage = active_gen->voltage();
        if (bus_voltage <= 1e-6f)
            bus_voltage = 1.0f;

        const float gen_available_W = active_gen->availablePowerW();
        const float gen_available_A = gen_available_W / bus_voltage;

        float demand_A = 0.0f;
        for (const auto& consumer : m_consumers) {
            if (consumer->isActive())
                demand_A += consumer->nominalPowerW() / bus_voltage;
        }

        if (demand_A > gen_available_A) {
            m_logger->warn(
                "DefaultPlatformPowerManager: generator [{}] cannot cover "
                "all consumers ({:.3f} A demand vs {:.3f} A available) – shedding",
                active_gen->name(),
                demand_A,
                gen_available_A);

            for (int group = 4; group >= 2; --group) {
                for (auto it = m_consumers.rbegin(); it != m_consumers.rend();
                     ++it) {
                    if (!(*it)->isActive() || (*it)->priority() != group)
                        continue;
                    demand_A -= (*it)->drawnCurrent();
                    (*it)->deactivate();
                    m_logger->warn(
                        "DefaultPlatformPowerManager: shed [{}] (priority {}), "
                        "remaining demand {:.3f} A",
                        (*it)->name(),
                        group,
                        demand_A);
                    if (demand_A <= gen_available_A)
                        goto done_shedding;
                }
            }
        done_shedding:;
        }
    } else {
        m_logger->info(
            "DefaultPlatformPowerManager: no power source – cutting all consumers");
        for (auto& consumer : m_consumers)
            consumer->deactivate();
    }
    return false;
}

void DefaultPlatformPowerManager::shedLoads(PowerLevel level)
{
    // DEPLETED is handled by handleDepleted(); treat as NORMAL here
    if (level == PowerLevel::NORMAL || level == PowerLevel::DEPLETED)
        return;

    // if a spare battery is available, no shedding needed
    for (int i = m_active_battery_index + 1;
         i < static_cast<int>(m_batteries.size());
         ++i) {
        if (!m_batteries[i]->isDepleted()) {
            m_logger->debug(
                "DefaultPlatformPowerManager: battery low but [{}] "
                "available, skipping shedding",
                m_batteries[i]->name());
            return;
        }
    }

    // if a generator is available to cover the load, no shedding needed
    auto active_gen = firstActiveGenerator();
    if (active_gen && !active_gen->isDepleted()) {
        m_logger->debug(
            "DefaultPlatformPowerManager: battery low, but generator [{}] available, "
            "skipping shedding",
            active_gen->name());
        return;
    }

    // WARN → shed priority 4; CRITICAL → shed priority 3 and below; one per
    // tick
    const int max_group = (level == PowerLevel::WARN) ? 4 : 3;
    for (int group = max_group; group >= 2; --group) {
        for (auto it = m_consumers.rbegin(); it != m_consumers.rend(); ++it) {
            if ((*it)->isActive() && (*it)->priority() == group) {
                (*it)->deactivate();
                m_logger->warn(
                    "DefaultPlatformPowerManager: shed consumer [{}] (priority {})",
                    (*it)->name(),
                    group);
                return;
            }
        }
    }
    m_logger->debug(
        "DefaultPlatformPowerManager: critical, only safety-critical consumers remain");
}

float DefaultPlatformPowerManager::computeChargeCurrentA(
    std::shared_ptr<Generator> gen,
    std::shared_ptr<Battery> bat,
    float safe_voltage) const
{
    if (!gen || !bat || gen->isDepleted())
        return 0.0f;
    if (bat->isDepleted())
        return 0.0f;
    if (bat->getStateOfCharge() >= 1.0f)
        return 0.0f;

    const float deficit = 1.0f - bat->getStateOfCharge();
    const float needed_W = deficit * gen->availablePowerW();
    const float max_charge_W = gen->availablePowerW() * 0.1f;
    return std::min(needed_W, max_charge_W) / safe_voltage;
}

void DefaultPlatformPowerManager::reactivateIfPossible(
    float available_w,
    float bus_voltage)
{
    const float available_a = available_w / bus_voltage;
    for (int priority = 1; priority <= 4; ++priority) {
        for (auto& consumer : m_consumers) {
            if (consumer->isActive() || consumer->priority() != priority)
                continue;
            const float cost_a = consumer->nominalPowerW() / bus_voltage;
            if (cost_a > available_a)
                continue;
            consumer->activate();
            m_logger->info(
                "DefaultPlatformPowerManager: reactivated consumer [{}] "
                "(priority {}) cost={:.3f} A available={:.3f} A",
                consumer->name(),
                priority,
                cost_a,
                available_a);
            return;
        }
    }
}

}  // namespace lotusim::gazebo
