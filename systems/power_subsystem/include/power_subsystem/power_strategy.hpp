#pragma once
#include <vector>
#include "power_subsystem/power_provider.hpp"
#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

class Battery;
class Generator;
class PowerConsumer;
class PowerProvider;

/**
 * @brief Owned by PowerManagerInstance, passed by ref to the strategy
 */
struct PowerStrategyContext {
    std::vector<Battery*>& batteries;
    std::vector<Generator*>& generators;
    std::vector<std::unique_ptr<PowerProvider>>& providers;
    std::vector<std::unique_ptr<PowerConsumer>>& consumers;

    int& activeBatteryIndex;
    bool& allBatteriesDepleted;

    float totalCurrentA;   // summed by Update() before calling strategy
    float busVoltage;      // active battery voltage (or 1.0 fallback)
};

/**
 * @brief Logic for load distribution, battery switching, and load shedding
 *        Inject via PowerManagerInstance::setStrategy()
 *
 * distributeLoad  : decide who supplies whom (battery / generator)
 * handleDepleted  : react to a depleted active battery
 * shedLoads       : shed / reactivate consumers by PowerLevel
 *
 * The default implementation (DefaultPowerStrategy) is the following logic:
 * 
 *   1. Push total_current to active battery via receiveLoad()
 *   2. Generators produce energy and charge the active or depleted battery
 *   3. If the active battery is depleted, it switches to the next battery. 
 *      If they are no more batteries left, the generator takes the load
 *   4. Switch behaviour:
 *        DEPLETED : switch to next battery, deactivate all if none left
 *        CRITICAL : shed priority 3 consumers and below
 *        WARN     : shed priority 4 consumers
 *        NORMAL   : no action
 */
class PowerStrategy {
public:
    virtual ~PowerStrategy() = default;

    /**
     * @brief distribute consumer demand across available sources
     *
     * Called after total_current is summed
     * Responsible for calling Battery::receiveLoad(), Generator::receiveLoad(),
     * and Battery::receiveCharge()
     *
     * @param context   power state for this vessel
     * @param dt        simulation timestep in seconds
     * @param logger    spdlog logger for this vessel (may be nullptr)
     */
    virtual void distributeLoad(
        PowerStrategyContext& context,
        float dt,
        std::shared_ptr<spdlog::logger> logger) = 0;

    /**
     * @brief Handle a depleted active battery
     *
     * Called when the active battery's PowerLevel == DEPLETED
     * Should advance context.activeBatteryIndex or set context.allBatteriesDepleted,
     * and shed consumers if required
     *
     * @return true  if a new battery was selected (simulation continues normally)
     *         false if all batteries are depleted (generator/emergency path)
     */
    virtual bool handleDepleted(
        PowerStrategyContext& context,
        float dt,
        std::shared_ptr<spdlog::logger> logger) = 0;

    /**
     * @brief shed or reactivate consumers based on PowerLevel
     */
    virtual void shedLoads(
        PowerStrategyContext& context,
        PowerLevel level,
        std::shared_ptr<spdlog::logger> logger) = 0;
};

} // namespace lotusim::gazebo