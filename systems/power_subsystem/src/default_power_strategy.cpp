/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "power_subsystem/default_power_strategy.hpp"
#include "power_subsystem/battery.hpp"
#include "power_subsystem/generator.hpp"
#include "power_subsystem/power_consumer.hpp"
#include <algorithm>

namespace lotusim::gazebo {

// helper
// find the first non-depleted generator
static Generator* firstActiveGenerator(std::vector<Generator*>& gens) {
    for (auto* g : gens)
        if (!g->isDepleted()) return g;
    return nullptr;
}

// Distribute load
void DefaultPowerStrategy::distributeLoad(
    PowerStrategyContext& context,
    float dt,
    std::shared_ptr<spdlog::logger> logger)
{
    Generator* active_gen = firstActiveGenerator(context.generators);

    if (!context.allBatteriesDepleted) {
        // normal: battery covers consumer demand
        context.batteries[context.activeBatteryIndex]->receiveLoad(context.totalCurrentA, dt);

        if (active_gen) {
            // Find a battery that needs charging
            Battery* charge_target = context.batteries[context.activeBatteryIndex];
            // if active battery is full, find one that needs charging
            if (charge_target->getStateOfCharge() >= 1.0f) {
                charge_target = nullptr;
                for (auto* bat : context.batteries) {
                    if (bat->getStateOfCharge() < 1.0f) { 
                        charge_target = bat;
                        break;
                    }
                }
            }
            if (charge_target) {
                const float charge_current = computeChargeCurrentA(active_gen, charge_target, context.busVoltage);
                if (charge_current > 1e-6f) {
                    active_gen->receiveLoad(charge_current, dt);
                    charge_target->receiveCharge(charge_current, dt);
                    if (logger)
                        logger->debug("DefaultPowerStrategy: generator [{}] charging "
                            "battery [{}] at {:.3f} A",
                            active_gen->name(), charge_target->name(), charge_current);
                }
                // else battery is full or generator has nothing to give
            } else {
                if (logger)
                    logger->debug("DefaultPowerStrategy: generator [{}] idle – "
                        "all batteries full", active_gen->name());
            }
        }
    } else {
        // emergency: generator covers consumers directly
        if (active_gen) {
            const float gen_available_W = active_gen->availablePowerW();
            const float gen_current = std::min(context.totalCurrentA, gen_available_W / context.busVoltage);
            active_gen->receiveLoad(gen_current, dt);

            if (logger)
                logger->debug("DefaultPowerStrategy: emergency – generator [{}] "
                    "covering {:.1f} W of {:.1f} W demand",
                    active_gen->name(), gen_current * context.busVoltage, context.totalCurrentA * context.busVoltage);

            const float headroom_W = gen_available_W - (gen_current - context.busVoltage);
            reactivateIfPossible(context, headroom_W, logger);
        } else {
            if (logger)
                logger->debug("DefaultPowerStrategy: no power source available");
        }
    }
}

// handle depleted
bool DefaultPowerStrategy::handleDepleted(
    PowerStrategyContext& context,
    float /*dt*/,
    std::shared_ptr<spdlog::logger> logger)
{
    // search from activeBatteryIndex forward,
    // advance only if a non-depleted battery is found further along
    for (int i = context.activeBatteryIndex;
         i < static_cast<int>(context.batteries.size()); ++i)
    {
        if (!context.batteries[i]->isDepleted()) {
            if (i != context.activeBatteryIndex){
                if (logger)
                    logger->info("DefaultPowerStrategy: switching to battery [{}]",
                        context.batteries[i]->name());
                context.activeBatteryIndex = i;
            }
            context.allBatteriesDepleted = false;
            return true;
        }
    }

    // All batteries depleted
    context.allBatteriesDepleted = true;
    if (logger)
        logger->warn("DefaultPowerStrategy: all batteries depleted");

    Generator* active_gen = firstActiveGenerator(context.generators);

    if (active_gen) {
        if (logger)
            logger->info("DefaultPowerStrategy: generator [{}] taking over consumer demand",
                active_gen->name());

        const float gen_available_W = active_gen->availablePowerW();
        const float gen_available_A = gen_available_W / context.busVoltage;
        float demandA = 0.0f;

        for (const auto& consumer : context.consumers) {
            if (consumer->isActive()){
                demandA += consumer->drawnCurrent();
            } 
        }
        if (demandA > gen_available_A) {
            if (logger)
                logger->warn("DefaultPowerStrategy: generator [{}] cannot cover "
                    "all consumers ({:.3f} A demand vs {:.3f} A available) – shedding",
                    active_gen->name(), demandA, gen_available_A);

            for (int group = 4; group >= 2; --group) {
                for (auto it = context.consumers.rbegin();
                     it != context.consumers.rend(); ++it)
                {
                    if (!(*it)->isActive()) continue;
                    if ((*it)->priority() != group) continue;

                    demandA -= (*it)->drawnCurrent();
                    (*it)->deactivate();
                    if (logger)
                        logger->warn("DefaultPowerStrategy: shed [{}] (priority {}), "
                            "remaining demand {:.3f} A",
                            (*it)->name(), group, demandA);

                    if (demandA <= gen_available_A) goto done_shedding;
                }
            }
            done_shedding:;
        }
    } else {
        if (logger)
            logger->info("DefaultPowerStrategy: no power source – cutting all consumers");
        for (auto& consumer : context.consumers) consumer->deactivate();
    }
    return false;
}

// shed load if low power 
void DefaultPowerStrategy::shedLoads(
    PowerStrategyContext& context,
    PowerLevel level,
    std::shared_ptr<spdlog::logger> logger)
{
    // Priority:
    //   1 = safety-critical         – never shed
    //   2 = operationally important – last resort only
    //   3 = mission systems
    //   4 = non-essential           – shed first
    //
    // DEPLETED is handled in handleDepleted() before this is called
    // If we arrive here with DEPLETED the battery just switched → treat as NORMAL

    if (level == PowerLevel::NORMAL || level == PowerLevel::DEPLETED)
        return;

    const int maxGroup = (level == PowerLevel::WARN) ? 4 : 3;

    for (int group = maxGroup; group >= 2; --group){
        for (auto it = context.consumers.rbegin(); it != context.consumers.rend(); ++it) {
            if ((*it)->isActive() && (*it)->priority() == group){
                (*it)->deactivate();
                if (logger)
                    logger->warn("DefaultPowerStrategy: shed consumer [{}] (priority {})",
                        (*it)->name(), group);
                return; //shed one per tick
            }
        }
    }   
    // only priority 1 consumers remain     
    if (logger)
        logger->warn("DefaultPowerStrategy: critical, only safety-critical consumers remain");
}

// helpers
float DefaultPowerStrategy::computeChargeCurrentA(
    Generator* gen, Battery* bat, float safe_voltage) const
{
    if (!gen || !bat || gen->isDepleted()) return 0.0f;
    if (bat->getStateOfCharge() >= 1.0f) return 0.0f;

    // generate only what the battery currently needs,
    // capped at 10% of the generator's rated output
    const float soc = bat->getStateOfCharge();
    const float deficit = 1.0f - soc;
    const float needed_W = deficit * gen->availablePowerW();
    const float max_charge_W = gen->availablePowerW() * 0.1f;
    const float charge_W = std::min(needed_W, max_charge_W);
    return charge_W / safe_voltage;
}

void DefaultPowerStrategy::reactivateIfPossible(
    PowerStrategyContext& context,
    float available_w,
    std::shared_ptr<spdlog::logger> logger)
{
    // reactivates highest priority first
    for (int priority = 1; priority <= 4; ++priority) {
        for (auto& consumer : context.consumers) {
            if (consumer->isActive()) continue;
            if (consumer->priority() != priority) continue;
            if (consumer->nominalPowerW() > available_w) continue;

            consumer->reactivate();
            if (logger)
                logger->info(
                    "DefaultPowerStrategy: reactivated consumer [{}] "
                    "(priority {}) available_w={:.1f}",
                    consumer->name(), priority, available_w);
            return;
        }
    }
}
} // namespace lotusim::gazebo