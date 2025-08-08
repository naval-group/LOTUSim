
#include "lotusim_sensor_plugin/lotusim_sensor_plugin.hpp"

namespace lotusim::sensor {

LotusimSensorPlugin::LotusimSensorPlugin() {}

void LotusimSensorPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    m_world_name = lotusim::common::getWorldName(_ecm);

    std::string logger_name = m_world_name + "_lotusim_sensors";

    m_logger =
        logger::createConsoleAndFileLogger(logger_name, logger_name + ".txt");

    m_ros_node = rclcpp::Node::make_shared(
        m_world_name + "_sensor_system",
        m_world_name);

    m_logger->info(
        "LotusimSensorPlugin::Configure: LotusimSensorPlugin successfully startup.");
}

void LotusimSensorPlugin::PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<
        gz::sim::components::CustomSensor,
        gz::sim::components::ParentEntity>(std::bind(
        &LotusimSensorPlugin::EachNew,
        this,
        &_ecm,
        std::placeholders::_1,
        std::placeholders::_2));

    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::CustomSensor *) -> bool {
            if (m_entity_sensor_map.erase(_entity) == 0) {
                m_logger->error(
                    "LotusimSensorPlugin::PreUpdate: Sensor entity [{}] removed but not found in system",
                    _entity);
            }
            return true;
        });
}

void LotusimSensorPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Only update and publish if not paused.
    if (!_info.paused) {
        for (auto &[entity, sensor] : m_entity_sensor_map) {
            auto world_pos = gz::sim::worldPose(entity, _ecm);
            sensor->NewPosition(world_pos.Pos());

            auto lat_long = gz::sim::sphericalCoordinates(entity, _ecm);
            if (lat_long) {
                sensor->NewLatLong(lat_long.value());
            }

            sensor->Update(_info, _ecm);
        }
    }
}

bool LotusimSensorPlugin::EachNew(
    gz::sim::EntityComponentManager *_ecm,
    const gz::sim::Entity &_entity,
    const gz::sim::components::CustomSensor *_custom)
{
    try {
        std::string model_name;
        gz::sim::Entity model_entity;

        auto model_name_opt = lotusim::common::getModelName(*_ecm, _entity);
        if (model_name_opt) {
            model_entity = model_name_opt->first;
            model_name = model_name_opt->second;
        }

        auto sensor_name =
            _ecm->Component<gz::sim::components::Name>(_entity)->Data();

        sdf::Sensor data = _custom->Data();
        auto type = gz::sensors::customType(data);
        std::unique_ptr<CustomSensor> sensor;
        // Add more sensor below
        if (type == "subsea_pressure") {
            m_logger->info(
                "LotusimSensorPlugin::subsea_pressure: Creating sensor [{}/{}]",
                model_name,
                sensor_name);
            sensor = CreateSensor<SubseaPressureSensor>(
                data,
                model_entity,
                _entity,
                model_name,
                sensor_name);
        } else if (type == "ais") {
            m_logger->info(
                "LotusimSensorPlugin::ais: Creating sensor [{}/{}]",
                model_name,
                sensor_name);
            sensor = CreateSensor<AISSensor>(
                data,
                model_entity,
                _entity,
                model_name,
                sensor_name);
        } else {
            return true;
        }
        auto child_link = _ecm->ChildrenByComponents(
            model_entity,
            gz::sim::components::Link());
        for (auto &&link : child_link) {
            auto name_opt = _ecm->Component<gz::sim::components::Name>(link);
            if (name_opt &&
                name_opt->Data().find("base_link") != std::string::npos) {
                gz::sim::Link _link(link);
                _link.EnableVelocityChecks(*_ecm);
                break;
            }
        }

        m_entity_sensor_map.insert(std::make_pair(_entity, std::move(sensor)));
        return true;
    } catch (...) {
        return true;
    }
}

}  // namespace lotusim::sensor

GZ_ADD_PLUGIN(
    lotusim::sensor::LotusimSensorPlugin,
    gz::sim::System,
    lotusim::sensor::LotusimSensorPlugin::ISystemConfigure,
    lotusim::sensor::LotusimSensorPlugin::ISystemPreUpdate,
    lotusim::sensor::LotusimSensorPlugin::ISystemPostUpdate)
