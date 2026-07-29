/*
 * Copyright (c) 2025 Naval Group
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_SCENARIO_MANAGER_HH_
#define LOTUSIM_SCENARIO_MANAGER_HH_

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_mas/entity_spawner.hpp"
#include "lotusim_mas/scenario_parser.hpp"
#include "lotusim_mas/scenario_types.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"
#include "lotusim_msgs/srv/string.hpp"

namespace lotusim::scenario {

/**
 * @brief Manages scenario in the simulation.
 *
 * Responsibilities:
 *  - Expose `launch_scenario` and `stop_scenario` ROS 2 services.
 *  - Parse the scenario YAML on launch.
 *  - Call EntitySpawner to spawn / despawn agents.
 *  - (Future) Set environment parameters via dedicated services.
 *
 * ScenarioManager does NOT implement gz::sim::System interfaces — it is a
 * plain helper owned by MAS.
 */
class ScenarioManager {
public:
    /**
     * @param node        Shared ROS node (owned by MAS).
     * @param spawner     Reference to the EntitySpawner (must outlive this).
     * @param logger      Shared spdlog logger.
     * @param ecm         Pointer to the Gazebo ECM (must outlive this).
     */
    ScenarioManager(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<lotusim::mas::EntitySpawner> spawner,
        std::shared_ptr<spdlog::logger> logger,
        gz::sim::EntityComponentManager* ecm);

    ~ScenarioManager() = default;

    // Non-copyable, movable
    ScenarioManager(const ScenarioManager&) = delete;
    ScenarioManager& operator=(const ScenarioManager&) = delete;
    ScenarioManager(ScenarioManager&&) = default;

    /**
     * @brief Currently active scenario, if any.
     */
    const std::optional<ScenarioConfig>& currentScenario() const
    {
        return m_current_scenario;
    }

    /**
     * @brief True when a scenario is running.
     */
    bool isRunning() const
    {
        return m_current_scenario.has_value();
    }

private:
    /**
     * @brief Handler for `launch_scenario` service.
     *
     * Expects request.data = absolute or relative path to the YAML file.
     * Spawns all agents defined in the scenario.
     */
    void handleLaunch(
        const std::shared_ptr<lotusim_msgs::srv::String::Request> request,
        std::shared_ptr<lotusim_msgs::srv::String::Response> response);

    /**
     * @brief Handler for `stop_scenario` service.
     *
     * Despawns all agents that were created by the active scenario.
     */
    void handleStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief Spawns every agent in cfg using m_spawner->
     * @return Number of agents successfully spawned.
     */
    int spawnAgents(const ScenarioConfig& cfg);

    /**
     * @brief Removes all agents that were recorded in m_spawned_names.
     */
    void despawnAgents();

    /**
     * @brief Build a MASCmd message from a parsed Agent struct.
     */
    lotusim_msgs::msg::MASCmd agentToMASCmd(const lotusim::Agent& agent) const;

    /**
     * @brief Update the world's SphericalCoordinates reference point from cfg.
     */
    void applyReferencePosition(const GeoPoint& ref);

private:
    std::shared_ptr<spdlog::logger> m_logger;

    rclcpp::Node::SharedPtr m_ros_node;

    std::vector<rclcpp::CallbackGroup::SharedPtr> m_callback_group;

    std::shared_ptr<lotusim::mas::EntitySpawner> m_spawner;

    gz::sim::EntityComponentManager* m_ecm{nullptr};

    std::optional<ScenarioConfig> m_current_scenario;

    rclcpp::Service<lotusim_msgs::srv::String>::SharedPtr m_launch_srv;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_stop_srv;
};

}  // namespace lotusim::scenario
#endif  // LOTUSIM_SCENARIO_MANAGER_HH_