/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_sensor_plugin/lotusim_sensor_plugin.hpp"

namespace lotusim::sensor {

LotusimSensorPlugin::LotusimSensorPlugin() {}

void LotusimSensorPlugin::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>&,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&)
{
    m_gz_node.Subscribe("/collision", &LotusimSensorPlugin::collisionCB, this);

    m_ecm = &_ecm;
    m_world_name = lotusim::common::getWorldName(_ecm);

    std::string logger_name = m_world_name + "_lotusim_sensors";

    m_logger =
        logger::createConsoleAndFileLogger(logger_name, logger_name + ".txt");

    m_ros_node = rclcpp::Node::make_shared(
        m_world_name + "_sensor_system",
        m_world_name);
    m_collision_pub =
        m_ros_node->create_publisher<lotusim_sensor_msgs::msg::Collisions>(
            "collisions",
            rclcpp::QoS(10));

    m_logger->info(
        "LotusimSensorPlugin::Configure: LotusimSensorPlugin successfully startup.");
}

void LotusimSensorPlugin::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    // {
    // if (!m_scene) {
    //     m_scene = gz::rendering::sceneFromFirstRenderEngine();
    //     return;
    // }

    // auto lidar_iter = m_lidar_tbc.begin();
    // while (lidar_iter != m_lidar_tbc.end()) {
    //     std::string sensor_name = "frigate::base_link::gpu_lidar";
    //     m_logger->info(
    //         "LotusimSensorPlugin::PreUpdate: Lidar Sensor entity [{}]
    //         [{}] detected", *lidar_iter, sensor_name);

    //     auto sensor = m_scene->SensorByName(sensor_name);
    //     if (!sensor) {
    //         m_logger->error(
    //             "Lidar Sensor entity [{}] [{}] could not be found in
    //             scene", *lidar_iter, sensor_name);
    //         lidar_iter++;
    //         continue;
    //     } else {
    //         m_logger->info("Sensor type: {}", typeid(*sensor).name());
    //     }

    //     gpuLidar =
    //         std::dynamic_pointer_cast<gz::rendering::GpuRays>(sensor);
    //     if (!gpuLidar) {
    //         m_logger->warn(
    //             "LotusimSensorPlugin::PreUpdate: Lidar Sensor entity [{}]
    //             [{}] unable to be linked", *lidar_iter, sensor_name);
    //         lidar_iter++;
    //         continue;
    //     }

    //     std::string vessel_name;
    //     auto vessel_name_opt =
    //         lotusim::common::getModelName(*m_ecm, *lidar_iter);
    //     if (vessel_name_opt) {
    //         vessel_name = vessel_name_opt->second;
    //     }

    //     auto sensor_pub =
    //         m_ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    //             vessel_name + "/" + "gpu_lidar" + "/" + "lidar",
    //             rclcpp::QoS(1));

    //     m_lidar_name[*lidar_iter] = sensor_name;
    //     m_lidar_pub_mapping[*lidar_iter] = sensor_pub;

    //     auto connection = gpuLidar->ConnectNewGpuRaysFrame(
    //         std::bind(
    //             &LotusimSensorPlugin::OnNewLidarFrame,
    //             this,
    //             *lidar_iter,
    //             std::placeholders::_1,
    //             std::placeholders::_2,
    //             std::placeholders::_3,
    //             std::placeholders::_4,
    //             std::placeholders::_5));
    //     m_gpu_rays_connection.push_back(connection);

    //     m_logger->info(
    //         "LotusimSensorPlugin::PreUpdate: Lidar Sensor entity [{}]
    //         [{}] ROS2 created", *lidar_iter, sensor_name);

    //     lidar_iter = m_lidar_tbc.erase(lidar_iter);
    // }
}

void LotusimSensorPlugin::OnNewLidarFrame(
    const gz::sim::Entity& _entity,
    const float* _scan,
    unsigned int _width,
    unsigned int _height,
    unsigned int _channels,
    const std::string& /*_format*/)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = common::generateHeaderMessage(m_current_time);
    msg.header.frame_id = m_lidar_name[_entity];  // or proper TF frame

    msg.height = _height;
    msg.width = _width;

    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.is_bigendian = false;

    msg.point_step = _channels * sizeof(float);
    msg.row_step = msg.point_step * _width;

    msg.is_dense = true;

    size_t total_bytes = _width * _height * _channels * sizeof(float);
    msg.data.resize(total_bytes);
    memcpy(msg.data.data(), _scan, total_bytes);

    m_lidar_pub_mapping[_entity]->publish(msg);
}

void LotusimSensorPlugin::collisionCB(const gz::msgs::Contacts& _msg)
{
    std::lock_guard<std::mutex> lock(m_collision_mutex);
    for (auto&& collision : _msg.contact()) {
        m_collision_graph.addPair(
            collision.collision1().id(),
            collision.collision2().id());
    }
}

void LotusimSensorPlugin::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    // TODO
    // Temporary measure to handle the lidar publish ros2 pointcloud
    // Need to optimised too.
    // The m_ecm held is shit
    // _ecm.EachNew<gz::sim::components::Sensor, gz::sim::components::GpuLidar>(
    //     [this](
    //         const gz::sim::Entity &_entity,
    //         const gz::sim::components::Sensor *_sensor,
    //         const gz::sim::components::GpuLidar *_gpuLidar) -> bool {
    //         m_lidar_tbc.push_back(_entity);
    //         return true;
    //     });

    /// Collision handling, to rewrite to detect at this level
    {
        std::vector<std::vector<uint64_t>> collisions;
        {
            std::lock_guard<std::mutex> lock(m_collision_mutex);
            collisions = m_collision_graph.getAllSets();
            m_collision_graph.clearGraph();
        }
        lotusim_sensor_msgs::msg::Collisions collisions_msg;
        for (auto&& collided_entities : collisions) {
            lotusim_sensor_msgs::msg::Collision collision;
            for (auto&& entity : collided_entities) {
                collision.entity.push_back(entity);
            }
            collisions_msg.collisions.push_back(collision);
        }
        m_collision_pub->publish(collisions_msg);
    }

    _ecm.EachNew<
        gz::sim::components::CustomSensor,
        gz::sim::components::ParentEntity>(std::bind(
        &LotusimSensorPlugin::EachNew,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    _ecm.EachRemoved<gz::sim::components::CustomSensor>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::CustomSensor*) -> bool {
            if (m_entity_sensor_map.erase(_entity) == 0) {
                m_logger->error(
                    "LotusimSensorPlugin::PreUpdate: Sensor entity [{}] removed but not found in system",
                    _entity);
            }
            return true;
        });

    // Only update and publish if not paused.
    if (!_info.paused) {
        m_current_time = _info.simTime;
        for (auto& [entity, sensor] : m_entity_sensor_map) {
            auto world_pos = gz::sim::worldPose(entity, _ecm);
            sensor->Position(world_pos.Pos());
            sensor->Orientation(world_pos.Rot());

            auto lat_long = gz::sim::sphericalCoordinates(entity, _ecm);
            if (lat_long) {
                sensor->LatLong(lat_long.value());
            }
            sensor->UpdateSensor(_info, _ecm);
        }
    }
}

bool LotusimSensorPlugin::EachNew(
    const gz::sim::Entity& _entity,
    const gz::sim::components::CustomSensor* _custom)
{
    try {
        std::string model_name;
        gz::sim::Entity model_entity;

        auto model_name_opt = lotusim::common::getModelName(*m_ecm, _entity);
        if (model_name_opt) {
            model_entity = model_name_opt->first;
            model_name = model_name_opt->second;
        }

        auto sensor_name =
            m_ecm->Component<gz::sim::components::Name>(_entity)->Data();

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
                "LotusimSensorPlugin::AIS: Creating sensor [{}/{}]",
                model_name,
                sensor_name);
            sensor = CreateSensor<AISSensor>(
                data,
                model_entity,
                _entity,
                model_name,
                sensor_name);
        } else if (type == "imu") {
            m_logger->info(
                "LotusimSensorPlugin::IMU: Creating sensor [{}/{}]",
                model_name,
                sensor_name);
            sensor = CreateSensor<IMUSensor>(
                data,
                model_entity,
                _entity,
                model_name,
                sensor_name);

        } else {
            return true;
        }
        auto child_link = m_ecm->ChildrenByComponents(
            model_entity,
            gz::sim::components::Link());
        for (auto&& link : child_link) {
            auto name_opt = m_ecm->Component<gz::sim::components::Name>(link);
            if (name_opt &&
                name_opt->Data().find("base_link") != std::string::npos) {
                gz::sim::Link _link(link);
                _link.EnableVelocityChecks(*m_ecm);
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
