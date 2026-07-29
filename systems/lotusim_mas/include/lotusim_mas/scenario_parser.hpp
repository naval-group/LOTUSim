/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_SCENARIO_PARSER_HH_
#define LOTUSIM_SCENARIO_PARSER_HH_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "lotusim_mas/scenario_types.hpp"

namespace lotusim::scenario {

/**
 * @brief Stateless YAML → ScenarioConfig parser.
 *
 * Parses the scenario YAML and, for each agent, converts the
 * `physics_interface` and `render_interface` YAML blocks directly into a
 * <lotus_param> XML string stored in Agent::lotus_param.
 * No intermediate structs are kept for those two sections.
 *
 * Currently, it only handles agents / platforms in the simulation.
 * Environment variables is to be added once enviornment system is formalized
 */
class ScenarioParser {
public:
    static std::optional<ScenarioConfig> fromFile(const std::string& path);
    static std::optional<ScenarioConfig> fromString(const std::string& yaml);

private:
    static std::optional<ScenarioConfig> parseRoot(const YAML::Node& root);

    static std::vector<Agent> parseAgents(const YAML::Node& node);
    static Agent parseAgent(const std::string& key, const YAML::Node& node);

    /**
     * @brief Build the full <lotus_param> XML string from the YAML nodes.
     *
     * Any node may be null/undefined — the corresponding XML section is
     * simply omitted in that case.
     */
    static std::string buildLotusParam(
        const YAML::Node& render_node,
        const YAML::Node& physics_node,
        const YAML::Node& waypoint_node);

    static void appendRenderInterface(std::string& xml, const YAML::Node& node);

    static void appendPhysicsEngineInterface(
        std::string& xml,
        const YAML::Node& node);

    static void appendWaypointFollower(
        std::string& xml,
        const YAML::Node& node);

    /** "underwater" → "Underwater", etc. */
    static std::string capitaliseDomain(const std::string& domain);

    template <typename T>
    static T
    getOrDefault(const YAML::Node& node, const std::string& key, T default_val);
};

}  // namespace lotusim::scenario
#endif  // LOTUSIM_SCENARIO_PARSER_HH_