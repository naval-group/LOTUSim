/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <string>
#include <unordered_map>
#include <rclcpp/executors.hpp>
#include <thread>

#include "lotusim_common/logger.hpp"
#include "power_subsystem/platform_power_manager_base.hpp"

namespace lotusim::gazebo {

/**
 * @brief world-level plugin managing power subsystems for all vessels
 *
 * Declared once in lotusim.world
 * Detects vessel spawns via EachNew<Model> and creates
 *  a PlatformPowerManager per vessel
 *
 * World SDF format (lotusim.world):
 *   <plugin filename="power_subsystem"
 *           name="lotusim::gazebo::PowerManager">
 *   </plugin>
 *
 * For models:
 * Lotusim param to handle the model's power system management type.
 * Default power management will be used if this param is not stated.
 * Check default power manager script for more info.
 * <lotus_param>
 *      <power_system>
 *          <power_management_type>default</power_management_type>
 *      </power_system>
 * </lotus_param>
 *
 * Power Provider
 * In models, there is statement for provider,
 * the type is defined in by interface
 * <lotusim_power>
 *    <name>main_battery</name>
 *    <type>simple_battery</type>
 *    <capacity_ah>100</capacity_ah>
 *    <initial_soc>0.8</initial_soc>
 *    <voltage_min>2.0</voltage_min>
 *    <voltage_nominal>16.0</voltage_nominal>
 *  </lotusim_power>
 *
 * Power Consumer
 * such as sensor or anything that takes power.
 * Type is defined based on interface
 * <lotusim_power>
 *   <type>sensor</type>
 *   <nominal_w>7.0</nominal_w>
 *   <priority>2</priority>
 * </lotusim_power>
 *
 */
class PowerManager : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemUpdate,
                     public gz::sim::ISystemPostUpdate {
public:
    PowerManager();
    ~PowerManager() override;

    /**
     * @brief World-level init: stores ECM, sets up logger and node
     */
    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    /**
     * @brief updates a vessel consumers once loaded in
     */
    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

    /**
     * @brief Detects vessel spawns/despawns and act on all instances
     */
    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief creates a PlatformPowerManager for a newly spawned ve ssel
     *        only vessels with at least one <lotusim_power> link are
     *        registered
     * @param _entity  model entity
     * @param _model   Model component
     * @param _ecm     ECM
     * returns true if instance was created successfully
     */
    bool loadVessel(
        const gz::sim::Entity& _entity,
        const gz::sim::components::ModelSdf* _model,
        gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief destroys the PlatformPowerManager for a removed vessel
     * @param _entity  Model entity of the despawned vessel
     * returns true if instance was found and removed
     */
    bool deleteVessel(const gz::sim::Entity& _entity);

private:
    std::shared_ptr<spdlog::logger> m_logger;
    std::string m_world_name;
    rclcpp::Node::SharedPtr m_ros_node;
    gz::sim::Entity m_entity;
    gz::sim::EntityComponentManager* m_ecm{nullptr};
    rclcpp::executors::MultiThreadedExecutor m_executor;
    std::thread m_spin_thread;

    /// One PlatformPowerManager per vessel, keyed by model entity
    std::unordered_map<
        gz::sim::Entity,
        std::unique_ptr<PlatformPowerManagerBase>>
        m_vessel_instances;
};

}  // namespace lotusim::gazebo