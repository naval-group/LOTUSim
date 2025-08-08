
#include "ais_sensor/ais_sensor.hpp"

namespace lotusim::sensor {

AISSensor::AISSensor(
    std::shared_ptr<spdlog::logger> logger,
    rclcpp::Node::SharedPtr node,
    const gz::sim::Entity &vessel_entity,
    const gz::sim::Entity &sensor_entity,
    const std::string &parent_name,
    const std::string &sensor_name)
    : CustomSensor(
          logger,
          node,
          vessel_entity,
          sensor_entity,
          parent_name,
          sensor_name)
    , m_update_period(std::chrono::seconds(2))
    , m_last_pub(std::chrono::seconds(0))
    , m_base_link(gz::sim::kNullEntity)
{
}
AISSensor::~AISSensor() = default;

bool AISSensor::CustomSensorLoad(const sdf::Sensor &_sdf)
{
    m_sensor_pub = m_ros_node->create_publisher<lotusim_sensor_msgs::msg::AIS>(
        m_vessel_name + "/" + m_sensor_name + "/" + "ais",
        rclcpp::QoS(10));
    return true;
}

bool AISSensor::Update(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    if (m_base_link == gz::sim::kNullEntity) {
        auto child_link = _ecm.ChildrenByComponents(
            m_vessel_entity,
            gz::sim::components::WorldLinearVelocity());
        for (auto &&link : child_link) {
            m_base_link = link;
            break;
        }
    }

    if (!EnableMeasurement(_info.realTime))
        return false;

    lotusim_sensor_msgs::msg::AIS msg;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(_info.realTime)
            .count();
    msg.header.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(simTimeNs % 1000000000);
    msg.header.frame_id = "world";
    msg.name = m_vessel_name;
    msg.longitude = m_lat_long.Y();
    msg.latitude = m_lat_long.X();

    auto vel_opt =
        _ecm.Component<gz::sim::components::WorldLinearVelocity>(m_base_link);

    if (vel_opt) {
        double vel = std::sqrt(
            std::pow(vel_opt->Data()[0], 2) + std::pow(vel_opt->Data()[1], 2));
        msg.sog = vel;
    }

    double angleRadians =
        atan2(vel_opt->Data()[0], vel_opt->Data()[1]);  // x is East, y is North
    double headingDegrees = angleRadians * 180.0 / M_PI;
    if (headingDegrees < 0) {
        headingDegrees += 360.0;
    }
    msg.true_heading = headingDegrees;

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.realTime;
    return true;
}

}  // namespace lotusim::sensor
