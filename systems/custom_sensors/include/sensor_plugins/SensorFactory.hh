
#ifndef __SENSOR_FACTORY_HH_
#define __SENSOR_FACTORY_HH_

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>
#include <string>
#include <unordered_map>
#include <utility>

#include "lotusim_common/logger.hpp"
#include "sensor_plugins/CustomSensor.hh"
#include "sensor_plugins/SubseaPressureSensor.hh"

namespace lotusim::gazebo {

/**
 * @brief Custom sensor factory
 * TODO: Move the AIS sensor to custom sensor.
 * Documenting and testing.
 *
 */
class SensorSystem : public gz::sim::System,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate {
public:
    SensorSystem();

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    std::unordered_map<gz::sim::Entity, std::shared_ptr<CustomSensor>>
        m_entitySensorMap;
};

}  // namespace lotusim::gazebo
#endif