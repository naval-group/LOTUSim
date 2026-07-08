/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_mas/scenario_manager.hpp"

#include "lotusim_common/common.hpp"

namespace lotusim::scenario {

ScenarioManager::ScenarioManager(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<lotusim::mas::EntitySpawner> spawner,
    std::shared_ptr<spdlog::logger> logger,
    gz::sim::EntityComponentManager* ecm)
    : m_logger{std::move(logger)}
    , m_ros_node{std::move(node)}
    , m_spawner{spawner}
    , m_ecm{ecm}
{
    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));
    m_launch_srv = m_ros_node->create_service<lotusim_msgs::srv::String>(
        "launch_scenario",
        std::bind(
            &ScenarioManager::handleLaunch,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        m_callback_group.back());

    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));
    m_stop_srv = m_ros_node->create_service<std_srvs::srv::Trigger>(
        "stop_scenario",
        std::bind(
            &ScenarioManager::handleStop,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        m_callback_group.back());

    m_logger->info("ScenarioManager: Services registered.");
}

void ScenarioManager::handleLaunch(
    const std::shared_ptr<lotusim_msgs::srv::String::Request> request,
    std::shared_ptr<lotusim_msgs::srv::String::Response> response)
{
    const char* scenario_env = std::getenv("LOTUSIM_SCENARIOS_PATH");
    if (!scenario_env) {
        m_logger->warn(
            "ScenarioManager::handleLaunch: Environment variable LOTUSIM_SCENARIOS_PATH not set.");
        scenario_env = "~/lotusim_ws/src/LOTUSim/assets/scenarios";
    }
    const std::string path = std::string(scenario_env) + "/" + request->data;

    if (m_current_scenario) {
        m_logger->warn(
            "ScenarioManager::handleLaunch: Scenario '{}' already running. "
            "Stop it first.",
            m_current_scenario->name);
        response->success = false;
        response->message =
            "A scenario is already running: " + m_current_scenario->name;
        return;
    }

    m_logger->info("ScenarioManager::handleLaunch: Loading '{}'", path);

    auto cfg = ScenarioParser::fromFile(path);
    if (!cfg) {
        m_logger->error(
            "ScenarioManager::handleLaunch: Failed to parse '{}'",
            path);
        response->success = false;
        response->message = "Failed to parse scenario file: " + path;
        return;
    }

    m_current_scenario = std::move(cfg);

    if (m_ecm) {
        applyReferencePosition(m_current_scenario->reference_position);
    }

    int spawned = spawnAgents(*m_current_scenario);

    m_logger->info(
        "ScenarioManager::handleLaunch: Scenario '{}' started. {}/{} agents spawned.",
        m_current_scenario->name,
        spawned,
        static_cast<int>(m_current_scenario->agents.size()));

    response->success = true;
    response->message = "Scenario launched: " + m_current_scenario->name;
}

void ScenarioManager::handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!m_current_scenario) {
        m_logger->warn("ScenarioManager::handleStop: No scenario is running.");
        response->success = false;
        response->message = "No scenario is currently running.";
        return;
    }

    m_logger->info(
        "ScenarioManager::handleStop: Stopping scenario '{}'",
        m_current_scenario->name);

    despawnAgents();
    m_current_scenario.reset();

    response->success = true;
    response->message = "Scenario stopped.";
}

int ScenarioManager::spawnAgents(const ScenarioConfig& cfg)
{
    int count = 0;
    for (const auto& agent : cfg.agents) {
        auto cmd = agentToMASCmd(agent);
        auto result = m_spawner->addEntity(cmd);
        if (result) {
            auto [entity, name] = *result;
            m_logger->info(
                "ScenarioManager::spawnAgents: Spawned '{}' (entity {})",
                name,
                entity);
            ++count;
        } else {
            m_logger->error(
                "ScenarioManager::spawnAgents: Failed to spawn agent '{}'",
                agent.name);
        }
    }
    return count;
}

void ScenarioManager::despawnAgents()
{
    m_spawner->deleteAllEntities();
}

lotusim_msgs::msg::MASCmd ScenarioManager::agentToMASCmd(
    const lotusim::Agent& agent) const
{
    lotusim_msgs::msg::MASCmd cmd;
    cmd.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
    cmd.model_name = agent.model;
    cmd.vessel_name = agent.name;

    cmd.geo_point.latitude = agent.position.latitude;
    cmd.geo_point.longitude = agent.position.longitude;
    cmd.geo_point.altitude = agent.position.altitude;
    cmd.heading = agent.heading;

    // heading → yaw (Z-up, East=0, right-hand rule)
    const double yaw = agent.heading * M_PI / 180.0;
    cmd.vessel_position.orientation.w = std::cos(yaw / 2.0);
    cmd.vessel_position.orientation.x = 0.0;
    cmd.vessel_position.orientation.y = 0.0;
    cmd.vessel_position.orientation.z = std::sin(yaw / 2.0);

    // lotus_param XML was built by ScenarioParser — pass it straight through.
    // MAS::addEntity expects it in sdf_string when model_name is set.
    cmd.sdf_string = agent.lotus_param;

    return cmd;
}

void ScenarioManager::applyReferencePosition(const GeoPoint& ref)
{
    gz::sim::Entity worldEntity = gz::sim::kNullEntity;
    m_ecm->Each<gz::sim::components::Name, gz::sim::components::World>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Name*,
            const gz::sim::components::World*) -> bool {
            worldEntity = _entity;
            return true;
        });

    auto* sphComp = m_ecm->Component<gz::sim::components::SphericalCoordinates>(
        worldEntity);
    if (sphComp == nullptr) {
        m_logger->warn(
            "ScenarioManager::applyReferencePosition: SphericalCoordinates "
            "component not found on world entity.");
        return;
    }

    sphComp->Data().SetLatitudeReference(
        gz::math::Angle(lotusim::common::degToRad(ref.latitude)));
    sphComp->Data().SetLongitudeReference(
        gz::math::Angle(lotusim::common::degToRad(ref.longitude)));
    sphComp->Data().SetElevationReference(ref.altitude);

    m_logger->info(
        "ScenarioManager::applyReferencePosition: World reference set to "
        "lat={:.6f} lon={:.6f} alt={:.2f}",
        ref.latitude,
        ref.longitude,
        ref.altitude);
}

}  // namespace lotusim::scenario