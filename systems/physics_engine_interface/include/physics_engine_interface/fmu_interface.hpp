/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_PHYSICS_ENGINE_INTERFACE_FMU_INTERFACE_HH_
#define LOTUSIM_PHYSICS_ENGINE_INTERFACE_FMU_INTERFACE_HH_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>

#include "physics_engine_interface/fmi2_slave.hpp"
#include "physics_engine_interface/physics_interface_base.hpp"

namespace lotusim::gazebo {

/**
 * @brief Vessel physics computed by an FMI 2.0 Co-Simulation FMU, loaded in
 * the simulator's process.
 *
 * Each step is one fmi2DoStep, the way XdynWebsocket sends one request to
 * xdyn: Gazebo's state goes in, the FMU's state comes out. The FMU follows
 * the names of lxdyn's FMU (xdyn_wrapper_fmi), <body> being the body's name:
 * - outputs <body>.x y z (NED position, m), u v w (FRD body velocities, m/s),
 *   p q r (FRD body rates, rad/s), qr qi qj qk (attitude quaternion);
 * - optional inputs <body>.reset_state (Boolean) and <body>.reset_x ...
 *   reset_qk: when present, Gazebo's state is imposed before every step, so
 *   teleports and domain changes carry over, as with xdyn-for-cs;
 * - the vessel's commands are the inputs whose description, or name, is the
 *   command, as xdyn names it: propeller(rpm) sets
 *   <body>.propeller.rpm; "(rpm)" commands are converted to rad/s, as for
 *   xdyn.
 *
 * Any model that keeps these names can be packed this way.
 *
 * SDF, in a domain block of <physics_engine_interface>:
 *   <interface_type>FMU</interface_type>
 *   <fmu>dtmb_hull/dtmb.fmu</fmu>   absolute, or under LOTUSIM_MODELS_PATH
 *                                   or GZ_SIM_RESOURCE_PATH
 *   <fmu_parameters>                optional, set before initialization
 *     <parameter name="xdyn.dt">0.2</parameter>
 *   </fmu_parameters>
 *
 * One FMU instance per vessel and domain; instances of different vessels
 * are stepped in parallel.
 */
class FmuInterface : public PhysicsInterfaceBase {
public:
    explicit FmuInterface(const std::string& world_name);
    ~FmuInterface() override = default;

    static std::shared_ptr<FmuInterface> createInterface(
        const std::string& world_name);
    static void resetInstance()
    {
        m_instance.reset();
    }

    bool configureInterface(
        const gz::sim::Entity& entity,
        const std::string& model_name,
        const sdf::ElementPtr sdf,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool removeInterface(
        const gz::sim::Entity& entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool activateInterface(
        const gz::sim::Entity& entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    bool deactivateInterface(
        const gz::sim::Entity& entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    std::optional<std::tuple<VesselInformation, DomainType>> getNewState(
        const gz::sim::Entity& entity,
        const VesselInformation& previous_state,
        float time_dif) override;

    std::string getURI(
        const gz::sim::Entity& entity,
        const DomainType& domain_type = DomainType::Unknown) override;

    /**
     * @brief Where an <fmu> path points: absolute, else the first match under
     * LOTUSIM_MODELS_PATH then each GZ_SIM_RESOURCE_PATH directory. Empty if
     * none exists.
     */
    static std::string resolveFmuPath(const std::string& path);

private:
    struct Instance {
        std::unique_ptr<Fmi2Slave> fmu;
        std::string path;
        std::string vessel;
        DomainType domain = DomainType::Unknown;
        std::string body;  ///< prefix of the state variables, "TestShip."
        bool can_reset = false;
        /// LOTUSim command (xdyn's name) -> the FMU input it sets
        std::map<std::string, std::string> inputs;
        std::set<std::string> unknown_commands;  ///< warned once each
        std::mutex step_mutex;
    };

    std::shared_ptr<Instance> current(const gz::sim::Entity& entity);

    static std::shared_ptr<FmuInterface> m_instance;

    std::mutex m_mutex;  ///< guards the maps, not the instances
    std::unordered_map<
        gz::sim::Entity,
        std::map<DomainType, std::shared_ptr<Instance>>>
        m_instances;
    std::unordered_map<gz::sim::Entity, DomainType> m_active;
};

}  // namespace lotusim::gazebo

#endif
