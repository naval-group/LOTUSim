/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_manager.hpp"

#include <gz/common/Console.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "lotusim_common/common.hpp"

namespace lotusim::gazebo {

PowerManager::PowerManager()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}
PowerManager::~PowerManager()
{
    m_logger->info(
        "PowerManager::~PowerManager: PowerManager successfully shutdown.");
}

void PowerManager::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& /*_sdf*/,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/)
{
    m_entity = _entity;
    m_ecm = &_ecm;

    m_world_name = lotusim::common::getWorldName(_ecm);

    const std::string logger_name = m_world_name + "_power_manager";
    m_logger =
        logger::createConsoleAndFileLogger(logger_name, logger_name + ".txt");

    // world-level ROS2 node used for publishing
    // per-vessel nodes are created inside each PlatformPowerManager
    m_ros_node = rclcpp::Node::make_shared(
        m_world_name + "_power_manager",
        m_world_name);

    m_logger->info(
        "PowerManager::Configure: power subsystem started for world [{}]",
        m_world_name);
}

void PowerManager::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    const float dt =
        static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    for (auto& [entity, instance] : m_vessel_instances) {
        instance->PostUpdate(dt);
    }
}

void PowerManager::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Detect new vessels ────────────────────────────────────────────
    _ecm.EachNew<gz::sim::components::Model, gz::sim::components::ModelSdf>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Model*,
            const gz::sim::components::ModelSdf* _model) -> bool {
            loadVessel(_entity, _model, _ecm);
            return true;
        });

    // ── Detect removed vessels ────────────────────────────────────────
    _ecm.EachRemoved<gz::sim::components::Model>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Model*) -> bool {
            deleteVessel(_entity);
            return true;
        });

    const float dt =
        static_cast<float>(std::chrono::duration<double>(_info.dt).count());

    // ── Tick all active vessel instances ──────────────────────────────
    for (auto& [entity, instance] : m_vessel_instances) {
        instance->Update(dt);
    }
}

// ============================================================
// Private helpers
// ============================================================

bool PowerManager::loadVessel(
    const gz::sim::Entity& _entity,
    const gz::sim::components::ModelSdf* _model_sdf,
    gz::sim::EntityComponentManager& _ecm)
{
    // get vessel name
    auto nameOpt = lotusim::common::getModelName(_ecm, _entity);
    if (!nameOpt) {
        m_logger->warn(
            "PowerManager::loadVessel: could not get name for entity [{}]",
            _entity);
        return false;
    }
    const std::string vesselName = nameOpt->second;

    sdf::Model data = _model_sdf->Data();
    sdf::ElementPtr sdfptr = data.Element();
    if (!sdfptr) {
        m_logger->error(
            "PlatformPowerManager [{}]: ModelSdf has no element",
            vesselName);
        return false;
    }

    sdf::ElementPtr rootEl = sdfptr->GetIncludeElement();
    if (!rootEl) {
        rootEl = sdfptr;
    }

    // skip if already registered
    if (m_vessel_instances.count(_entity)) {
        m_logger->warn(
            "PowerManager::loadVessel: vessel [{}] already registered",
            vesselName);
        return false;
    }
    if (!_model_sdf) {
        m_logger->error(
            "PlatformPowerManager [{}]: no ModelSdf component found",
            vesselName);
        return false;
    }

    // Getting power management type for the ship
    PlatformPowerManagerType power_management_type =
        PlatformPowerManagerType::DEFAULT;
    // The lotus param will either be include statement or part of the
    // model
    if (rootEl->HasElement("lotus_param") &&
        rootEl->GetElement("lotus_param")->HasElement("power_system") &&
        rootEl->GetElement("lotus_param")
            ->GetElement("power_system")
            ->HasElement("power_management_type")) {
        power_management_type = platformPowerManagerTypeFromString(
            rootEl->GetElement("lotus_param")
                ->GetElement("power_system")
                ->Get<std::string>("power_management_type"));
    } else {
        m_logger->debug(
            "PowerManager::loadVessel [{}]: no <lotusim_power> tag found, skipping PowerManager creation",
            vesselName);
        return false;
    }

    auto vesselNode = m_ros_node->create_sub_node(vesselName + "_power");

    auto instance = PlatformPowerManagerBase::create(
        power_management_type,
        _entity,
        vesselName,
        vesselNode,
        rootEl);

    m_logger->info(
        "PowerManager::loadVessel: registered vessel [{}]",
        vesselName);

    m_vessel_instances.emplace(_entity, std::move(instance));

    return true;
}

bool PowerManager::deleteVessel(const gz::sim::Entity& _entity)
{
    auto it = m_vessel_instances.find(_entity);
    if (it == m_vessel_instances.end()) {
        return false;
    }

    m_logger->info(
        "PowerManager::deleteVessel: removing vessel [{}]",
        it->second->vesselName());

    m_vessel_instances.erase(it);
    return true;
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::PowerManager,
    gz::sim::System,
    lotusim::gazebo::PowerManager::ISystemConfigure,
    lotusim::gazebo::PowerManager::ISystemUpdate,
    lotusim::gazebo::PowerManager::ISystemPostUpdate)