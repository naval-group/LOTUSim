/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/kinematic_interface.hpp"

#include <cmath>

namespace lotusim::gazebo {

using json = nlohmann::json;

std::shared_ptr<KinematicInterface> KinematicInterface::m_instance = nullptr;

std::shared_ptr<KinematicInterface> KinematicInterface::getInstance()
{
    if (m_instance == nullptr) {
        m_instance = std::make_shared<KinematicInterface>();
    }
    return m_instance;
}

KinematicInterface::KinematicInterface()
    : PhysicsInterfaceBase("KinematicInterface")
{
}

bool KinematicInterface::createConnection(
    const gz::sim::Entity& _entity,
    const std::string& _name,
    const sdf::ElementPtr _sdf)
{
    // The element handed in is the domain block (surface/underwater/aerial).
    // Report that same domain from getNewState so the plugin's transition
    // logic stays a no-op for kinematic vessels.
    DomainType domain = DomainType::Surface;
    if (_sdf) {
        std::string element_name = lotusim::common::toUpper(_sdf->GetName());
        if (element_name == "UNDERWATER") {
            domain = DomainType::Underwater;
        } else if (element_name == "AERIAL") {
            domain = DomainType::Aerial;
        } else {
            domain = DomainType::Surface;
        }
    }

    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
    m_entity_domain[_entity] = domain;
    if (m_logger) {
        m_logger->info(
            "KinematicInterface::createConnection: {} registered as {} kinematic vessel.",
            _name,
            DomainTypeToStringMap[domain]);
    }
    return true;
}

bool KinematicInterface::removeConnection(const gz::sim::Entity& _entity)
{
    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
    m_entity_domain.erase(_entity);
    return true;
}

std::optional<std::tuple<VesselInformation, DomainType>>
KinematicInterface::getNewState(
    const gz::sim::Entity& _entity,
    const VesselInformation& previous_state,
    float time_dif)
{
    DomainType domain;
    {
        std::shared_lock<std::shared_mutex> lock(m_variable_mutex);
        auto domain_it = m_entity_domain.find(_entity);
        if (domain_it == m_entity_domain.end()) {
            return std::nullopt;
        }
        domain = domain_it->second;
    }

    // Body-frame velocity command set by the remote guidance node, defaulting
    // to a full stop when no command has been received yet.
    double u = 0.0;  // forward speed (m/s)
    double w = 0.0;  // yaw rate (rad/s)
    if (m_vessels_cmd_map_ptr) {
        auto cmd_it = m_vessels_cmd_map_ptr->find(_entity);
        if (cmd_it != m_vessels_cmd_map_ptr->end() && !cmd_it->second.empty()) {
            try {
                json cmd = json::parse(cmd_it->second);
                u = cmd.value("u", 0.0);
                w = cmd.value("w", 0.0);
            } catch (const std::exception& e) {
                if (m_logger) {
                    m_logger->warn(
                        "KinematicInterface::getNewState: bad cmd_string for entity {}: {}",
                        _entity,
                        e.what());
                }
            }
        }
    }

    // Integrate with Gazebo's own time step (ms -> s): no clock divergence.
    const double dt_s = time_dif / 1000.0;

    VesselInformation new_state = previous_state;
    new_state.entity = _entity;
    new_state.time = previous_state.time + dt_s;

    const double yaw = previous_state.pose.Yaw();

    gz::math::Pose3d pose = previous_state.pose;
    pose.SetX(pose.X() + u * std::cos(yaw) * dt_s);
    pose.SetY(pose.Y() + u * std::sin(yaw) * dt_s);
    pose.Rot().SetFromEuler(pose.Roll(), pose.Pitch(), yaw + w * dt_s);
    new_state.pose = pose;

    // World-frame velocities for the velocity components written to the ECM.
    new_state.lin_vel.Set(u * std::cos(yaw), u * std::sin(yaw), 0.0);
    new_state.ang_vel.Set(0.0, 0.0, w);

    logEngineState(new_state, domain);

    return std::make_optional(std::make_tuple(new_state, domain));
}

bool KinematicInterface::activateConnection(const gz::sim::Entity&)
{
    return true;
}

bool KinematicInterface::deactivateConnection(const gz::sim::Entity&)
{
    return true;
}

std::string KinematicInterface::getURI(const gz::sim::Entity&)
{
    return "kinematic";
}

}  // namespace lotusim::gazebo
