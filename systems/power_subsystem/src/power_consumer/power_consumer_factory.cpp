#include "power_subsystem/power_consumer/power_consumer.hpp"
#include "power_subsystem/power_consumer/sensor_power_consumer.hpp"
#include "power_subsystem/power_consumer/thruster_power_consumer.hpp"

namespace lotusim::gazebo {

PowerConsumer::CreateResult PowerConsumer::createFromSdf(
    const std::string& name,
    const sdf::ElementPtr& sdf,
    rclcpp::Node::SharedPtr node,
    const std::string& vessel_name,
    std::shared_ptr<spdlog::logger> logger)
{
    if (!sdf->HasElement("lotusim_power")) {
        logger->debug(
            "PowerConsumer::createFromSdf [{}]: consumer [{}] has no "
            "<lotusim_power> -> skipping",
            vessel_name,
            name);
        return {nullptr, ConsumerType{}};
    }

    const sdf::ElementPtr powerEl = sdf->GetElement("lotusim_power");
    const std::string powerTypeStr =
        powerEl->Get<std::string>("type", "").first;
    const float nominalW = powerEl->Get<float>("nominal_w", 1.0f).first;
    const int priority = powerEl->Get<int>("priority", 3).first;

    if (powerTypeStr.empty()) {
        logger->warn(
            "PowerConsumer::createFromSdf [{}]: consumer [{}] has <lotusim_power> "
            "but missing <type> -> skipping",
            vessel_name,
            name);
        return {nullptr, ConsumerType{}};
    }

    const auto typeOpt = consumerTypeFromString(powerTypeStr);
    if (!typeOpt) {
        logger->warn(
            "PowerConsumer::createFromSdf [{}]: unknown type '{}' on "
            "consumer [{}] -> skipping",
            vessel_name,
            powerTypeStr,
            name);
        return {nullptr, ConsumerType{}};
    }

    const ConsumerType type = *typeOpt;

    switch (type) {
        case ConsumerType::Sensor: {
            auto consumer = std::make_shared<SensorPowerConsumer>(
                name,
                powerEl,
                node,
                logger,
                nominalW,
                priority);
            consumer->setServiceName(vessel_name);
            logger->info(
                "PowerConsumer::createFromSdf [{}]: registered sensor consumer "
                "[{}] (type={}) nominal_w={} priority={}",
                vessel_name,
                name,
                toString(type),
                nominalW,
                priority);
            return {std::move(consumer), type};
        }
        case ConsumerType::Thruster: {
            auto consumer = std::make_shared<ThrusterPowerConsumer>(
                name,
                powerEl,
                node,
                logger,
                nominalW,
                priority);
            logger->info(
                "PowerConsumer::createFromSdf [{}]: registered thruster consumer "
                "[{}] (type={}) nominal_w={} priority={}",
                vessel_name,
                name,
                toString(type),
                nominalW,
                priority);
            return {std::move(consumer), type};
        }
        default:
            return {nullptr, ConsumerType::Unknown};
    }
    return {nullptr, type};
}

}  // namespace lotusim::gazebo
