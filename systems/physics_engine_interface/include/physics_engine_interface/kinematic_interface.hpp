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

#include <atomic>
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
 * { "u": <forward speed m/s>, "w": <yaw rate rad/s>, "vz": <vertical rate m/s> }
 * ```
 * `vz` is optional (defaults to 0) -- most marine Kinematic vehicles never
 * set it. It exists for domains with real vertical motion, e.g. an aerial
 * vehicle in Kinematic mode.
 *
 * The motion model mirrors the legacy WaypointFollowerPlugin integration,
 * plus a world-frame vertical rate:
 * ```
 * x   += u * cos(yaw) * dt
 * y   += u * sin(yaw) * dt
 * z   += vz * dt
 * yaw += w * dt
 * ```
 * Roll and pitch are preserved (no attitude dynamics).
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

    bool configureInterface(
        const gz::sim::Entity& _entity,
        const std::string& _name,
        const sdf::ElementPtr _sdf,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool removeInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool activateInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool deactivateInterface(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    std::string getURI(
        const gz::sim::Entity& _entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    /**
     * @brief Set a world-frame (ENU) ocean current applied to every kinematic
     * vessel's position update, in addition to its own commanded velocity.
     *
     * Fed by PhysicsInterfacePlugin from the `ocean_current` ROS topic. This
     * is a uniform current (no depth or spatial variation) meant to perturb
     * Kinematic-connected vehicles and props (e.g. mines) without requiring
     * xdyn. Aerial entities are exempt.
     */
    void setCurrent(double x, double y);

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

    /**
     * @brief World-frame (ENU) ocean current, m/s. Plain atomics: read by
     * every vessel's getNewState (possibly concurrently, one instance shared
     * across all kinematic vessels), written rarely from the ROS callback. A
     * torn read across x/y for a single integration step is harmless for a
     * cosmetic current, so no need to route this through m_variable_mutex.
     */
    std::atomic<double> m_current_x{0.0};
    std::atomic<double> m_current_y{0.0};
};

}  // namespace lotusim::gazebo

#endif
