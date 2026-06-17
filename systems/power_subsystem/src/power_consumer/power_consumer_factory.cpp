#include "power_subsystem/power_consumer/power_consumer.hpp"
namespace lotusim::gazebo {

static CreateResult PowerConsumer::createFromSdf(
    const std::string& name,
    const sdf::ElementPtr& sdf,
    rclcpp::Node::SharedPtr node,
    const std::string& vessel_name,
    std::shared_ptr<spdlog::logger> logger);

{
    if (!sensorEl->HasElement("lotusim_power")) {
        m_logger->debug(
            "PlatformPowerManager [{}]: sensor [{}] has no "
            "<lotusim_power> -> skipping",
            m_vessel_name,
            sensorName);
        return;
    }

    const sdf::ElementPtr powerEl = sensorEl->GetElement("lotusim_power");
    const std::string powerType =
        powerEl->Get<std::string>("power_type", "").first;
    const float nominalW = powerEl->Get<float>("nominal_w", 1.0f).first;
    const int priority = powerEl->Get<int>("priority", 3).first;

    if (powerType.empty()) {
        m_logger->warn(
            "PlatformPowerManager [{}]: sensor [{}] has <lotusim_power> "
            "but missing <power_type> -> skipping",
            m_vessel_name,
            sensorName);
        return;
    }

    if (powerType == "sensor") {
        auto consumer = std::make_unique<SensorPowerConsumer>(
            sensorName,
            nominalW,
            priority,
            sensorEl,
            m_node,
            _ecm);
        consumer->setServiceName(m_vessel_name);
        m_logger->info(
            "PlatformPowerManager [{}]: registered sensor consumer "
            "[{}] (type={}) nominal_w={} priority={}",
            m_vessel_name,
            sensorName,
            sensorType,
            nominalW,
            priority);
        m_consumers.push_back(std::move(consumer));
    } else {
        m_logger->warn(
            "PlatformPowerManager [{}]: unknown power_type '{}' on "
            "sensor [{}] -> skipping",
            m_vessel_name,
            powerType,
            sensorName);
    }
}  // namespace lotusim::gazebo

}  // namespace lotusim::gazebo