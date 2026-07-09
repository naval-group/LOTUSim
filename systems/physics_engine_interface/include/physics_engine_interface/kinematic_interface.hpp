/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_KINEMATIC_INTERFACE_HH_
#define LOTUSIM_KINEMATIC_INTERFACE_HH_

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <unordered_map>

#include "physics_engine_interface/physics_interface_base.hpp"

namespace lotusim::gazebo {

/**
 * @brief Kinematic physics interface.
 *
 * This interface does NOT call any external physics engine. It integrates the
 * vessel pose from a body-frame velocity command using Gazebo's own simulation
 * time step (passed in as @p time_dif), so there is no clock divergence.
 *
 * The guidance (waypoint selection, heading/speed control) is expected to run
 * on a remote agent node, which publishes a velocity set-point on
 * `/<world>/vessel_cmd_array`. The shared command map (filled by
 * PhysicsInterfacePlugin) carries, per entity, a JSON string:
 * ```json
 * { "u": <forward speed m/s>, "w": <yaw rate rad/s> }
 * ```
 *
 * The motion model mirrors the legacy WaypointFollowerPlugin integration:
 * ```
 * x   += u * cos(yaw) * dt
 * y   += u * sin(yaw) * dt
 * yaw += w * dt
 * ```
 * Only x, y and yaw are updated; z, roll and pitch are preserved (2D motion).
 *
 * ### Example configuration
 * ```xml
 * <lotus_param>
 *   <physics_engine_interface>
 *     <surface>
 *       <connection_type>Kinematic</connection_type>
 *     </surface>
 *     <init_state>Surface</init_state>
 *   </physics_engine_interface>
 * </lotus_param>
 * ```
 */
class KinematicInterface : public PhysicsInterfaceBase {
public:
    KinematicInterface();

    /**
     * @brief Static accessor returning the shared singleton instance.
     *
     * One instance manages every kinematic vessel in the world (state is kept
     * per entity), matching the pattern used by the other interfaces.
     */
    static std::shared_ptr<KinematicInterface> getInstance();

    std::optional<std::tuple<VesselInformation, DomainType>> getNewState(
        const gz::sim::Entity& _entity,
        const VesselInformation& previous_state,
        float time_dif) override;

    bool createConnection(
        const gz::sim::Entity& _entity,
        const std::string& _name,
        const sdf::ElementPtr _sdf) override;

    bool removeConnection(const gz::sim::Entity& _entity) override;

    bool activateConnection(const gz::sim::Entity& _entity) override;

    bool deactivateConnection(const gz::sim::Entity& _entity) override;

    std::string getURI(const gz::sim::Entity& _entity) override;

private:
    static std::shared_ptr<KinematicInterface> m_instance;

    mutable std::shared_mutex m_variable_mutex;

    /**
     * @brief Domain reported by getNewState for each entity.
     *
     * Derived from the SDF element name (surface/underwater/aerial) so the
     * plugin's transition logic stays a no-op for kinematic vessels.
     */
    std::unordered_map<gz::sim::Entity, DomainType> m_entity_domain;
};

}  // namespace lotusim::gazebo

#endif
