
#ifndef __LOTUSIM_SENSOR_PLUGIN_HH_
#define __LOTUSIM_SENSOR_PLUGIN_HH_

#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>
#include <string>
#include <unordered_map>
#include <utility>

#include "ais_sensor/ais_sensor.hpp"
#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subsea_pressure_sensor/subsea_pressure_sensor.hh"

namespace lotusim::sensor {

/**
 * @brief Custom sensor factory
 * TODO: Move the AIS sensor to custom sensor.
 * Documenting and testing.
 *
 */
class LotusimSensorPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate {
public:
    LotusimSensorPlugin();

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    template <typename SensorType>
    std::unique_ptr<SensorType> CreateSensor(
        const sdf::Sensor &_sdf,
        const gz::sim::Entity &vessel_entity,
        const gz::sim::Entity &sensor_entity,
        const std::string &parent_name,
        const std::string &sensor_name)
    {
        auto sensor = std::make_unique<SensorType>(
            m_logger,
            m_ros_node,
            vessel_entity,
            sensor_entity,
            parent_name,
            sensor_name);

        if (nullptr == sensor) {
            gzerr << "Failed to create sensor [" << _sdf.Name() << "] of type["
                  << _sdf.TypeStr() << "]" << std::endl;
            return nullptr;
        }

        if (!sensor->Load(_sdf)) {
            gzerr << "Failed to load sensor [" << _sdf.Name() << "] of type["
                  << _sdf.TypeStr() << "]" << std::endl;
            return nullptr;
        }

        if (!sensor->Init()) {
            gzerr << "Failed to initialize sensor [" << _sdf.Name()
                  << "] of type[" << _sdf.TypeStr() << "]" << std::endl;
            return nullptr;
        }

        return sensor;
    }

    bool EachNew(
        gz::sim::EntityComponentManager *_ecm,
        const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom);

private:
    std::string m_world_name;

    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    rclcpp::Node::SharedPtr m_ros_node;

    std::unordered_map<gz::sim::Entity, std::shared_ptr<CustomSensor>>
        m_entity_sensor_map;
};

}  // namespace lotusim::sensor
#endif