/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "lotusim_mas/scenario_parser.hpp"

#include <cctype>
#include <unordered_map>

namespace lotusim::scenario {

std::optional<ScenarioConfig> ScenarioParser::fromFile(const std::string& path)
{
    try {
        return parseRoot(YAML::LoadFile(path));
    } catch (const YAML::Exception& e) {
        fprintf(
            stderr,
            "[ScenarioParser] YAML error in '%s': %s\n",
            path.c_str(),
            e.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        fprintf(
            stderr,
            "[ScenarioParser] Error loading '%s': %s\n",
            path.c_str(),
            e.what());
        return std::nullopt;
    }
}

std::optional<ScenarioConfig> ScenarioParser::fromString(
    const std::string& yaml)
{
    try {
        return parseRoot(YAML::Load(yaml));
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "[ScenarioParser] YAML parse error: %s\n", e.what());
        return std::nullopt;
    }
}

std::optional<ScenarioConfig> ScenarioParser::parseRoot(const YAML::Node& root)
{
    ScenarioConfig cfg;

    cfg.name = getOrDefault<std::string>(root, "name", "");
    cfg.time_of_day = getOrDefault<std::string>(root, "time_of_day", "12:00");
    cfg.date = getOrDefault<std::string>(root, "date", "");

    if (root["reference_position"]) {
        const auto& rp = root["reference_position"];
        cfg.reference_position.latitude =
            getOrDefault<double>(rp, "latitude", 0.0);
        cfg.reference_position.longitude =
            getOrDefault<double>(rp, "longitude", 0.0);
        cfg.reference_position.altitude =
            getOrDefault<double>(rp, "altitude", 0.0);
    }

    if (root["agents"])
        cfg.agents = parseAgents(root["agents"]);

    return cfg;
}

std::vector<Agent> ScenarioParser::parseAgents(const YAML::Node& node)
{
    std::vector<Agent> agents;
    if (!node.IsMap())
        return agents;
    for (const auto& pair : node)
        agents.push_back(parseAgent(pair.first.as<std::string>(), pair.second));
    return agents;
}

Agent ScenarioParser::parseAgent(const std::string& key, const YAML::Node& node)
{
    Agent agent;
    agent.name = key;
    agent.model = getOrDefault<std::string>(node, "model", "");
    agent.heading = getOrDefault<double>(node, "heading", 0.0);

    if (node["position"]) {
        const auto& p = node["position"];
        agent.position.latitude = getOrDefault<double>(p, "latitude", 0.0);
        agent.position.longitude = getOrDefault<double>(p, "longitude", 0.0);
        agent.position.altitude = getOrDefault<double>(p, "altitude", 0.0);
    }

    // Build lotus_param XML directly from the two raw YAML nodes.
    // Neither section is stored as a struct — the XML string is the only
    // output.
    agent.lotus_param = buildLotusParam(
        node["render_interface"],
        node["physics_interface"],
        node["waypoint_interface"]);

    return agent;
}

std::string ScenarioParser::buildLotusParam(
    const YAML::Node& render_node,
    const YAML::Node& physics_node,
    const YAML::Node& waypoint_node)
{
    std::string xml = "<lotus_param>\n";

    if (render_node && render_node.IsMap())
        appendRenderInterface(xml, render_node);

    if (physics_node && physics_node.IsMap())
        appendPhysicsEngineInterface(xml, physics_node);

    if (waypoint_node && waypoint_node.IsMap())
        appendWaypointFollower(xml, waypoint_node);

    xml += "</lotus_param>";
    return xml;
}

void ScenarioParser::appendRenderInterface(
    std::string& xml,
    const YAML::Node& node)
{
    const bool enabled = getOrDefault<bool>(node, "enabled", true);
    const std::string renderer_type =
        getOrDefault<std::string>(node, "renderer_type", "");

    xml += "    <render_interface>\n";
    xml += "        <publish_render>" +
           std::string(enabled ? "true" : "false") + "</publish_render>\n";
    xml += "        <renderer_type_name>" + renderer_type +
           "</renderer_type_name>\n";
    xml += "    </render_interface>\n";
}

void ScenarioParser::appendPhysicsEngineInterface(
    std::string& xml,
    const YAML::Node& node)
{
    xml += "    <physics_engine_interface>\n";

    // Emit domains in a fixed order so underwater always precedes surface/air.
    static const std::vector<std::string> domain_order = {
        "underwater",
        "surface",
        "air"};

    // Build a lookup: domain name → YAML node from the domains sequence.
    std::unordered_map<std::string, YAML::Node> domain_map;
    if (node["domains"] && node["domains"].IsSequence()) {
        for (const auto& item : node["domains"]) {
            YAML::Node d = item;
            const std::string key = getOrDefault<std::string>(d, "domain", "");
            if (!key.empty())
                domain_map[key] = d;
        }
    }

    for (const auto& domain_key : domain_order) {
        auto it = domain_map.find(domain_key);
        if (it == domain_map.end())
            continue;

        const YAML::Node& d = it->second;
        const YAML::Node& params = d["interface_params"];

        const std::string interface_type =
            getOrDefault<std::string>(d, "interface_type", "");
        const std::string uri =
            params ? getOrDefault<std::string>(params, "uri", "") : "";

        xml += "        <" + domain_key + ">\n";
        xml += "            <interface_type>" + interface_type +
               "</interface_type>\n";
        xml += "            <uri>" + uri + "</uri>\n";

        if (params && params["thrusters"] && params["thrusters"].IsSequence()) {
            xml += "            <thrusters>\n";
            int idx = 1;
            for (const auto& t : params["thrusters"]) {
                const std::string name =
                    getOrDefault<std::string>(t, "name", "");
                xml += "                <thrusters" + std::to_string(idx++) +
                       ">" + name + "</thrusters" + std::to_string(idx - 1) +
                       ">\n";
            }
            xml += "            </thrusters>\n";
        }

        xml += "        </" + domain_key + ">\n";
    }

    // init_state: capitalise to match XML convention (Underwater, Surface, Air)
    const std::string init =
        capitaliseDomain(getOrDefault<std::string>(node, "init_domain", ""));
    xml += "        <init_state>" + init + "</init_state>\n";

    xml += "    </physics_engine_interface>\n";
}

void ScenarioParser::appendWaypointFollower(
    std::string& xml,
    const YAML::Node& node)
{
    if (!getOrDefault<bool>(node, "enabled", true))
        return;

    const bool loop = getOrDefault<bool>(node, "loop", false);
    const std::string mode = getOrDefault<std::string>(node, "mode", "");

    xml += "    <waypoint_follower>\n";
    xml += "        <follower>\n";
    xml += "            <loop>" + std::string(loop ? "true" : "false") +
           "</loop>\n";

    if (mode == "waypoints" && node["waypoints"] &&
        node["waypoints"].IsSequence()) {
        xml += "            <waypoints>\n";
        for (const auto& wp : node["waypoints"]) {
            const double lat = getOrDefault<double>(wp, "lat", 0.0);
            const double lng = getOrDefault<double>(wp, "lng", 0.0);
            xml += "                <waypoint>" + std::to_string(lat) + " " +
                   std::to_string(lng) + "</waypoint>\n";
        }
        xml += "            </waypoints>\n";
    } else if (mode == "line" && node["line"] && node["line"].IsMap()) {
        const YAML::Node& line = node["line"];
        const double direction = getOrDefault<double>(line, "direction", 0.0);
        const double length = getOrDefault<double>(line, "length", 0.0);
        xml += "            <line>\n";
        xml += "                <direction>" + std::to_string(direction) +
               "</direction>\n";
        xml +=
            "                <length>" + std::to_string(length) + "</length>\n";
        xml += "            </line>\n";
    } else if (mode == "circle" && node["circle"] && node["circle"].IsMap()) {
        const YAML::Node& circle = node["circle"];
        const double radius = getOrDefault<double>(circle, "radius", 0.0);
        xml += "            <circle>\n";
        xml +=
            "                <radius>" + std::to_string(radius) + "</radius>\n";
        xml += "            </circle>\n";
    }

    xml += "        </follower>\n";
    xml += "    </waypoint_follower>\n";
}

std::string ScenarioParser::capitaliseDomain(const std::string& domain)
{
    if (domain.empty())
        return domain;
    std::string out = domain;
    out[0] =
        static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

template <typename T>
T ScenarioParser::getOrDefault(
    const YAML::Node& node,
    const std::string& key,
    T default_val)
{
    if (node && node[key]) {
        try {
            return node[key].as<T>();
        } catch (const YAML::Exception& e) {
            fprintf(
                stderr,
                "[ScenarioParser] Bad value for key '%s': %s\n",
                key.c_str(),
                e.what());
        }
    }
    return default_val;
}

}  // namespace lotusim::scenario