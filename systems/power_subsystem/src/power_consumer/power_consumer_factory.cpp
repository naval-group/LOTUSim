#include "power_subsystem/power_consumer/power_consumer.hpp"
#include "power_subsystem/power_consumer/sensor_power_consumer.hpp"
#include "power_subsystem/power_consumer/thruster_power_consumer.hpp"

namespace lotusim::gazebo {

PowerConsumer::CreateResult PowerConsumer::createFromSdf(
    const std::string& consumer_name,
    const std::string& vessel_name,
    const sdf::ElementPtr& sdf,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<spdlog::logger> logger)
{
    if (!sdf->HasElement("lotusim_power")) {
        logger->debug(
            "PowerConsumer::createFromSdf : consumer [{},{}] has no "
            "<lotusim_power> -> skipping",
            vessel_name,
            consumer_name);
        return {nullptr, ConsumerType{}};
    }

    const sdf::ElementPtr powerEl = sdf->GetElement("lotusim_power");
    const std::string powerTypeStr =
        powerEl->Get<std::string>("type", "").first;

    if (powerTypeStr.empty()) {
        logger->warn(
            "PowerConsumer::createFromSdf : consumer [{},{}] has <lotusim_power> "
            "but missing <type> -> skipping",
            vessel_name,
            consumer_name);
        return {nullptr, ConsumerType{}};
    }

    const auto typeOpt = consumerTypeFromString(powerTypeStr);
    if (!typeOpt) {
        logger->warn(
            "PowerConsumer::createFromSdf : unknown type '{}' on "
            "consumer [{},{}] -> skipping",
            powerTypeStr,
            vessel_name,
            consumer_name);
        return {nullptr, ConsumerType{}};
    }

    const ConsumerType type = *typeOpt;

    switch (type) {
        case ConsumerType::Sensor: {
            auto consumer = std::make_shared<SensorPowerConsumer>(
                consumer_name,
                vessel_name,
                powerEl,
                node,
                logger);
            logger->info(
                "PowerConsumer::createFromSdf: registered sensor consumer "
                "[{},{}] (type={})",
                consumer_name,
                vessel_name,
                toString(type));
            return {std::move(consumer), type};
        }
        case ConsumerType::Thruster: {
            auto consumer = std::make_shared<ThrusterPowerConsumer>(
                consumer_name,
                vessel_name,
                powerEl,
                node,
                logger);
            logger->info(
                "PowerConsumer::createFromSdf : registered thruster consumer "
                "[{},{}] (type={})",
                consumer_name,
                vessel_name,
                toString(type));
            return {std::move(consumer), type};
        }
        default:
            return {nullptr, ConsumerType::Unknown};
    }
    return {nullptr, type};
}

}  // namespace lotusim::gazebo
