#include "subsea_pressure_sensor/subsea_pressure_sensor.hh"

namespace lotusim::sensor {

SubseaPressureSensor::SubseaPressureSensor(
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
{
}

SubseaPressureSensor::~SubseaPressureSensor() {}

bool SubseaPressureSensor::CustomSensorLoad(const sdf::Sensor &_sdf)
{
    sdf::ElementPtr _sdfptr = _sdf.Element();
    GetSDFParam<double>(_sdfptr, "m_saturation", m_saturation, 3000);
    GetSDFParam<bool>(_sdfptr, "estimate_depth_on", m_estimate_depth, false);
    GetSDFParam<double>(
        _sdfptr,
        "standard_pressure",
        m_standard_pressure,
        101.325);
    GetSDFParam<double>(_sdfptr, "kPa_per_meter", m_kPa_per_m, 9.80638);
    m_sensor_pub =
        m_ros_node->create_publisher<lotusim_sensor_msgs::msg::PressureDepth>(
            m_vessel_name + "/" + m_sensor_name + "/pressure_sensor",
            rclcpp::QoS(10));
    return true;
}

bool SubseaPressureSensor::Update(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Need to rewrite
    if (!EnableMeasurement(_info.realTime))
        return false;

    double depth = std::abs(m_position.Z());
    double pressure = m_standard_pressure;
    if (depth >= 0) {
        // Convert depth to pressure
        pressure += depth * m_kPa_per_m;
    }

    pressure += GetGaussianNoise(m_noise_amp);
    double inferredDepth = 0.0;
    // Estimate depth, if enabled
    if (m_estimate_depth) {
        inferredDepth = (pressure - m_standard_pressure) / m_kPa_per_m;
    }

    lotusim_sensor_msgs::msg::PressureDepth msg;
    msg.depth = inferredDepth;
    msg.pressure = pressure;
    msg.stddev = m_noise_sigma;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(_info.realTime)
            .count();
    msg.header.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(simTimeNs % 1000000000);
    msg.header.frame_id = "world";

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.realTime;
    return true;
}

}  // namespace lotusim::sensor
