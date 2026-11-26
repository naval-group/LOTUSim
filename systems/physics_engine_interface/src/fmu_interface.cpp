/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/fmu_interface.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

namespace lotusim::gazebo {

std::shared_ptr<FmuInterface> FmuInterface::m_instance = nullptr;

namespace {

const char* const STATES[13] =
    {"x", "y", "z", "u", "v", "w", "p", "q", "r", "qr", "qi", "qj", "qk"};

bool endsWith(const std::string& s, const std::string& suffix)
{
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// lxdyn's FMU names its variables after the body, TestShip.x ...
// TestShip.qk: the body is the prefix under which the 13 states are all Real
// outputs. Empty when no prefix or several do.
std::string bodyPrefix(const Fmi2Slave& fmu)
{
    const auto& vars = fmu.variables();
    std::vector<std::string> found;
    for (const auto& [name, var] : vars) {
        if (!endsWith(name, ".x")) {
            continue;
        }
        const std::string prefix = name.substr(0, name.size() - 1);
        bool complete = true;
        for (const char* s : STATES) {
            const auto it = vars.find(prefix + s);
            complete = complete && it != vars.end() &&
                       it->second.type == Fmi2Slave::Type::Real &&
                       it->second.causality == "output";
        }
        if (complete) {
            found.push_back(prefix);
        }
    }
    return found.size() == 1 ? found.front() : "";
}

}  // namespace

FmuInterface::FmuInterface(const std::string& world_name)
    : PhysicsInterfaceBase("FmuInterface", world_name)
{
}

std::shared_ptr<FmuInterface> FmuInterface::createInterface(
    const std::string& world_name)
{
    if (m_instance == nullptr) {
        m_instance = std::make_shared<FmuInterface>(world_name);
    }
    return m_instance;
}

std::string FmuInterface::resolveFmuPath(const std::string& path)
{
    namespace fs = std::filesystem;
    if (path.empty()) {
        return "";
    }
    if (fs::path(path).is_absolute()) {
        return fs::exists(path) ? path : "";
    }
    std::vector<std::string> roots;
    for (const char* var : {"LOTUSIM_MODELS_PATH", "GZ_SIM_RESOURCE_PATH"}) {
        const char* value = std::getenv(var);
        if (!value) {
            continue;
        }
        std::stringstream list(value);
        std::string root;
        while (std::getline(list, root, ':')) {
            if (!root.empty()) {
                roots.push_back(root);
            }
        }
    }
    for (const auto& root : roots) {
        const fs::path candidate = fs::path(root) / path;
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }
    return "";
}

bool FmuInterface::configureInterface(
    const gz::sim::Entity& entity,
    const std::string& name,
    const sdf::ElementPtr sdf,
    const DomainType& domain_type)
{
    if (domain_type == DomainType::Unknown) {
        m_logger->error(
            "FmuInterface::configureInterface: Model {} configured without a domain: no physics update for it",
            name);
        return false;
    }
    if (!sdf || !sdf->HasElement("fmu")) {
        m_logger->error(
            "FmuInterface::configureInterface: No <fmu> for {}",
            name);
        return false;
    }
    const std::string given = sdf->Get<std::string>("fmu");
    const std::string path = resolveFmuPath(given);
    if (path.empty()) {
        m_logger->error(
            "FmuInterface::configureInterface: {}: FMU '{}' not found (absolute, or under LOTUSIM_MODELS_PATH or GZ_SIM_RESOURCE_PATH)",
            name,
            given);
        return false;
    }

    auto instance = std::make_shared<Instance>();
    instance->path = path;
    instance->vessel = name;
    instance->domain = domain_type;
    try {
        instance->fmu = std::make_unique<Fmi2Slave>(
            path,
            name + "_" + DomainTypeToStringMap[domain_type],
            m_logger);
        instance->body = bodyPrefix(*instance->fmu);
        if (instance->body.empty()) {
            m_logger->error(
                "FmuInterface::configureInterface: {}: FMU '{}' has no single body with the Real outputs <body>.x ... <body>.qk (x y z u v w p q r qr qi qj qk)",
                name,
                path);
            return false;
        }
        const std::string& body = instance->body;
        instance->can_reset = instance->fmu->hasVariable(body + "reset_state");
        for (const char* s : STATES) {
            instance->can_reset =
                instance->can_reset &&
                instance->fmu->hasVariable(body + "reset_" + s);
        }
        // LOTUSim sends xdyn's command names, propeller(rpm), which lxdyn's
        // FMU gives as the description of TestShip.propeller.rpm. An input
        // named as the command itself is taken too.
        for (const auto& [vname, var] : instance->fmu->variables()) {
            if (var.causality != "input" || var.type != Fmi2Slave::Type::Real ||
                vname.rfind(body + "reset_", 0) == 0) {
                continue;
            }
            instance->inputs[vname] = vname;
            if (!var.description.empty()) {
                instance->inputs.emplace(var.description, vname);
            }
        }
        if (!instance->can_reset) {
            m_logger->warn(
                "FmuInterface::configureInterface: {}: FMU '{}' has no reset_state inputs: it keeps its own state, teleports and domain changes do not reach it",
                name,
                path);
        }

        if (sdf->HasElement("fmu_parameters")) {
            for (auto p = sdf->GetElement("fmu_parameters")->GetFirstElement();
                 p;
                 p = p->GetNextElement()) {
                if (p->GetName() != "parameter" || !p->HasAttribute("name")) {
                    continue;
                }
                const std::string pname =
                    p->GetAttribute("name")->GetAsString();
                instance->fmu->setFromString(pname, p->Get<std::string>());
            }
        }
    } catch (const FmuError& e) {
        m_logger->error(
            "FmuInterface::configureInterface: {}: {}",
            name,
            e.what());
        return false;
    }

    // Initial commands, as XdynWebsocket seeds them, so that switching a
    // vessel between the two interfaces changes nothing else: each
    // <thrusters> entry at 50 rpm, P/D 0.79, beta 0; each <control_surfaces>
    // signal at 0. Only the FMU's own inputs are seeded.
    if (m_models_cmd_map_ptr) {
        nlohmann::json initial = nlohmann::json::object();
        auto seed = [&](const std::string& signal, double value) {
            if (instance->inputs.count(signal)) {
                initial[signal] = value;
            }
        };
        if (sdf->HasElement("thrusters")) {
            for (auto t = sdf->GetElement("thrusters")->GetFirstElement(); t;
                 t = t->GetNextElement()) {
                const std::string thruster = t->Get<std::string>();
                seed(thruster + "(rpm)", 50.0);
                seed(thruster + "(P/D)", 0.79);
                seed(thruster + "(beta)", 0.0);
            }
        }
        if (sdf->HasElement("control_surfaces")) {
            for (auto c =
                     sdf->GetElement("control_surfaces")->GetFirstElement();
                 c;
                 c = c->GetNextElement()) {
                seed(c->Get<std::string>(), 0.0);
            }
        }
        if (!initial.empty()) {
            (*m_models_cmd_map_ptr)[entity] = initial.dump();
        }
    }

    std::unique_lock<std::mutex> lock(m_mutex);
    m_instances[entity][domain_type] = instance;
    m_logger->info(
        "FmuInterface::configureInterface: {} {} computed by FMU '{}' ({})",
        name,
        DomainTypeToStringMap[domain_type],
        instance->fmu->modelName(),
        path);
    return true;
}

bool FmuInterface::removeInterface(
    const gz::sim::Entity& entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if (domain_type == DomainType::Unknown) {
        m_instances.erase(entity);
        m_active.erase(entity);
        return true;
    }
    auto it = m_instances.find(entity);
    if (it != m_instances.end()) {
        it->second.erase(domain_type);
        if (it->second.empty()) {
            m_instances.erase(it);
        }
    }
    const auto active = m_active.find(entity);
    if (active != m_active.end() && active->second == domain_type) {
        m_active.erase(active);
    }
    return true;
}

bool FmuInterface::activateInterface(
    const gz::sim::Entity& entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    const auto it = m_instances.find(entity);
    if (it == m_instances.end() || !it->second.count(domain_type)) {
        return false;
    }
    m_active[entity] = domain_type;
    return true;
}

bool FmuInterface::deactivateInterface(
    const gz::sim::Entity& entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    const auto active = m_active.find(entity);
    if (active != m_active.end() &&
        (domain_type == DomainType::Unknown || active->second == domain_type)) {
        m_active.erase(active);
    }
    return true;
}

std::shared_ptr<FmuInterface::Instance> FmuInterface::current(
    const gz::sim::Entity& entity)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    const auto active = m_active.find(entity);
    if (active == m_active.end()) {
        return nullptr;
    }
    const auto it = m_instances.find(entity);
    if (it == m_instances.end()) {
        return nullptr;
    }
    const auto domain = it->second.find(active->second);
    return domain == it->second.end() ? nullptr : domain->second;
}

std::optional<std::tuple<VesselInformation, DomainType>>
FmuInterface::getNewState(
    const gz::sim::Entity& entity,
    const VesselInformation& previous_state,
    float time_diff)
{
    const auto instance = current(entity);
    if (!instance) {
        return std::nullopt;
    }
    std::string commands;
    if (m_models_cmd_map_ptr) {
        const auto it = m_models_cmd_map_ptr->find(entity);
        if (it != m_models_cmd_map_ptr->end()) {
            commands = it->second;
        }
    }

    std::unique_lock<std::mutex> step_lock(instance->step_mutex);
    Fmi2Slave& fmu = *instance->fmu;
    const double t = previous_state.time;
    const double h = time_diff / 1000.0;
    try {
        if (!fmu.initialized()) {
            fmu.initialize(t);
        }
        if (instance->can_reset) {
            // lxdyn's FMU only resets when reset_state goes from false to
            // true, as a master sets every input before each step.
            fmu.setBoolean(instance->body + "reset_state", false);
            const VesselInformation ned = previous_state.to_xdyn();
            const double values[13] = {
                ned.pose.Pos().X(),
                ned.pose.Pos().Y(),
                ned.pose.Pos().Z(),
                ned.lin_vel.X(),
                ned.lin_vel.Y(),
                ned.lin_vel.Z(),
                ned.ang_vel.X(),
                ned.ang_vel.Y(),
                ned.ang_vel.Z(),
                ned.pose.Rot().W(),
                ned.pose.Rot().X(),
                ned.pose.Rot().Y(),
                ned.pose.Rot().Z()};
            for (int i = 0; i < 13; ++i) {
                fmu.setReal(instance->body + "reset_" + STATES[i], values[i]);
            }
            fmu.setBoolean(instance->body + "reset_state", true);
        }
        if (!commands.empty()) {
            const auto json = nlohmann::json::parse(commands, nullptr, false);
            if (json.is_object()) {
                for (const auto& [key, value] : json.items()) {
                    if (!value.is_number()) {
                        continue;
                    }
                    const auto input = instance->inputs.find(key);
                    if (input == instance->inputs.end()) {
                        if (instance->unknown_commands.insert(key).second) {
                            m_logger->warn(
                                "FmuInterface::getNewState: {}: FMU '{}' has no input '{}', command ignored",
                                instance->vessel,
                                fmu.modelName(),
                                key);
                        }
                        continue;
                    }
                    double v = value.get<double>();
                    // As for xdyn: LOTUSim speaks rpm, the model SI.
                    if (endsWith(key, "(rpm)")) {
                        v *= 2.0 * M_PI / 60.0;
                    }
                    fmu.setReal(input->second, v);
                }
            }
        }
        fmu.doStep(t, h);

        double s[13];
        for (int i = 0; i < 13; ++i) {
            s[i] = fmu.getReal(instance->body + STATES[i]);
        }
        VesselInformation next = VesselInformation::from_xdyn(
            {s[0], s[1], s[2]},
            gz::math::Quaterniond(s[9], s[10], s[11], s[12]),
            {s[3], s[4], s[5]},
            {s[6], s[7], s[8]});
        next.time = t + h;
        next.entity = entity;
        logEngineState(next, instance->domain, instance->vessel);

        // As for xdyn: the domain follows the altitude.
        const double altitude = -s[2];
        DomainType domain = DomainType::Surface;
        if (altitude >= 10.0) {
            domain = DomainType::Aerial;
        } else if (altitude <= -10.0) {
            domain = DomainType::Underwater;
        }
        return std::make_tuple(next, domain);
    } catch (const FmuError& e) {
        m_logger->error(
            "FmuInterface::getNewState: {}: {}",
            instance->vessel,
            e.what());
        return std::nullopt;
    }
}

std::string FmuInterface::getURI(
    const gz::sim::Entity& entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    const auto it = m_instances.find(entity);
    if (it == m_instances.end()) {
        return "";
    }
    DomainType domain = domain_type;
    if (domain == DomainType::Unknown) {
        const auto active = m_active.find(entity);
        if (active == m_active.end()) {
            return "";
        }
        domain = active->second;
    }
    const auto instance = it->second.find(domain);
    return instance == it->second.end() ? ""
                                        : "fmu://" + instance->second->path;
}

}  // namespace lotusim::gazebo
