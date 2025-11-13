/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#ifndef LOTUSIM_SENSOR_PLUGIN_HH_
#define LOTUSIM_SENSOR_PLUGIN_HH_

#include <gz/msgs/contacts.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderEngineManager.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sensors/GpuLidarSensor.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <list>
#include <sdf/Sensor.hh>
#include <string>
#include <unordered_map>
#include <utility>

#include "ais_sensor/ais_sensor.hpp"
#include "imu_sensor/imu_sensor.hpp"
#include "lotusim_common/common.hpp"
#include "lotusim_common/entity_group.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/collisions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) final;

    void Update(
        const gz::sim::UpdateInfo&,
        gz::sim::EntityComponentManager& _ecm);

    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) final;

private:
    template <typename SensorType>
    std::unique_ptr<SensorType> CreateSensor(
        const sdf::Sensor& _sdf,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name)
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
        const gz::sim::Entity& _entity,
        const gz::sim::components::CustomSensor* _custom);

    void OnNewLidarFrame(
        const gz::sim::Entity& _entity,
        const float* _scan,
        unsigned int _width,
        unsigned int _height,
        unsigned int _channels,
        const std::string& /*_format*/);

    /**
     * @brief Collision callback from Gazebo transport
     *
     * @param _msg Contacts message
     */
    void collisionCB(const gz::msgs::Contacts& _msg);

private:
    std::string m_world_name;
    gz::rendering::RenderEngine* m_engine;
    gz::rendering::ScenePtr m_scene;
    gz::sim::EntityComponentManager* m_ecm;

    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    rclcpp::Node::SharedPtr m_ros_node;

    std::unordered_map<gz::sim::Entity, std::shared_ptr<CustomSensor>>
        m_entity_sensor_map;

    // Temp measure for lidar ros implementation
    std::chrono::steady_clock::duration m_current_time;
    std::unordered_map<
        gz::sim::Entity,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        m_lidar_pub_mapping;

    // Lidar to be created
    std::vector<std::shared_ptr<gz::common::Connection>> m_gpu_rays_connection;

    std::list<gz::sim::Entity> m_lidar_tbc;
    std::shared_ptr<gz::rendering::GpuRays> gpuLidar;
    std::unordered_map<gz::sim::Entity, std::string> m_lidar_name;

    /// Temporary measure for collision handling
    std::mutex m_collision_mutex;
    gz::transport::Node m_gz_node;
    lotusim::common::EntityGraph m_collision_graph;
    rclcpp::Publisher<lotusim_sensor_msgs::msg::Collisions>::SharedPtr
        m_collision_pub;
};

}  // namespace lotusim::sensor
#endif