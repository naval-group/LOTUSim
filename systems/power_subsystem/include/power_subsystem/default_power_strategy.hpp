#pragma once
#include "power_subsystem/power_strategy.hpp"

namespace lotusim::gazebo {

/**
 * @brief Default logic for the load covering and shredding
 *
 * Normal mode  : battery covers demand; generator charges battery
 * Emergency    : generator covers demand directly; consumers shed by priority
 * Shedding     : WARN → shed priority 4; CRITICAL → shed priority 3 and below
 * Reactivation : one consumer per tick when generator has headroom
 */
class DefaultPowerStrategy : public PowerStrategy {
public:
    void distributeLoad(
        PowerStrategyContext& context,
        float dt,
        std::shared_ptr<spdlog::logger> logger) override;

    bool handleDepleted(
        PowerStrategyContext& context,
        float dt,
        std::shared_ptr<spdlog::logger> logger) override;

    void shedLoads(
        PowerStrategyContext& context,
        PowerLevel level,
        std::shared_ptr<spdlog::logger> logger) override;

private:
    float computeChargeCurrentA(
        Generator* gen,
        Battery* bat,
        float safeVoltage) const;

    void reactivateIfPossible(
        PowerStrategyContext& context,
        float available_w,
        std::shared_ptr<spdlog::logger> logger);
};

} // namespace lotusim::gazebo