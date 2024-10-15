
#include "sensor_plugins/SensorFactory.hh"

namespace lotusim::gazebo {

//////////////////////////////////////////////////
SensorSystem::SensorSystem()
{
    m_logger = logger::createConsoleAndFileLogger(
        "custom_sensor",
        "custom_sensor.txt");
}

//////////////////////////////////////////////////
void SensorSystem::PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<
        gz::sim::components::CustomSensor,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::CustomSensor *_custom,
            const gz::sim::components::ParentEntity *_parent) -> bool {
            // Get sensor's scoped name without the world
            auto sensorScopedName = gz::sim::removeParentScope(
                gz::sim::scopedName(_entity, _ecm, "::", false),
                "::");
            sdf::Sensor data = _custom->Data();
            data.SetName(sensorScopedName);

            // To change to add sensor type into topic name
            if (data.Topic().empty()) {
                std::string topic = scopedName(_entity, _ecm);
                data.SetTopic(topic);
            }

            gz::sensors::SensorFactory sensorFactory;

            auto type = gz::sensors::customType(data);
            std::unique_ptr<CustomSensor> sensor;
            // Add more sensor below
            if (type == "subsea_pressure") {
                sensor = sensorFactory.CreateSensor<SubseaPressureSensor>(data);
                m_logger->info(
                    "SensorSystem::PreUpdate: Creating sensor [{}]",
                    sensorScopedName);
            } else {
                return true;
            }

            // Set sensor parent
            auto parentName =
                _ecm.Component<gz::sim::components::Name>(_parent->Data())
                    ->Data();
            sensor->SetParent(parentName);

            // Set topic on Gazebo
            _ecm.CreateComponent(
                _entity,
                gz::sim::components::SensorTopic(sensor->Topic()));

            m_entitySensorMap.insert(
                std::make_pair(_entity, std::move(sensor)));
            return true;
        });
}

//////////////////////////////////////////////////
void SensorSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Only update and publish if not paused.
    if (!_info.paused) {
        for (auto &[entity, sensor] : m_entitySensorMap) {
            sensor->NewPosition(gz::sim::worldPose(entity, _ecm).Pos());
            sensor->Update(_info.simTime);
        }
    }

    RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void SensorSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::CustomSensor *) -> bool {
            if (m_entitySensorMap.erase(_entity) == 0) {
                m_logger->error(
                    "SensorSystem::PreUpdate: Internal error, missing odometer for entity [{}]",
                    _entity);
            }
            return true;
        });
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::SensorSystem,
    gz::sim::System,
    lotusim::gazebo::SensorSystem::ISystemPreUpdate,
    lotusim::gazebo::SensorSystem::ISystemPostUpdate)
