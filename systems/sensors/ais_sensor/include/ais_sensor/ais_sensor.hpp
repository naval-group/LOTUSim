#ifndef __AIS_SENSOR_HPP__
#define __AIS_SENSOR_HPP__

#include <cmath>
#include <gz/sim/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>

#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/ais.hpp"

namespace lotusim::sensor {

class AISSensor : public CustomSensor {
public:
    AISSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity &vessel_entity,
        const gz::sim::Entity &sensor_entity,
        const std::string &parent_name,
        const std::string &sensor_name);

    ~AISSensor();

    virtual bool Update(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor &_sdf) final;

private:
    // Sensor params
    std::chrono::steady_clock::duration m_update_period;
    std::chrono::steady_clock::duration m_last_pub;
    gz::sim::Entity m_base_link;
    std::vector<std::pair<std::string, gz::math::Pose3d>> m_pose;
    rclcpp::Publisher<lotusim_sensor_msgs::msg::AIS>::SharedPtr m_sensor_pub;
};

}  // namespace lotusim::sensor
#endif