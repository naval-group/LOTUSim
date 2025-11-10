/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_GAZEBO_PHYSICS_INTERFACE_PLUGIN_HPP_
#define LOTUSIM_GAZEBO_PHYSICS_INTERFACE_PLUGIN_HPP_

#include <future>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/World.hh>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lotusim_msgs/msg/vessel_cmd.hpp"
#include "lotusim_msgs/msg/vessel_cmd_array.hpp"
#include "physics_engine_interface/physics_interface_base.hpp"
#include "physics_engine_interface/ros2_interface.hpp"
#include "physics_engine_interface/xdyn_websocket.hpp"

namespace lotusim::gazebo {

/**
 * @brief This is a plugin for interfacing with external physics library
 * It will:
 * - Handle all vehicles in the world that require interfacing with
 *   physics calculation by asynchronously sending out requests
 * - Handle the model transition between different boundaries and swap server
 *
 * IT WILL NOT HANDLE receiving vessel commands and calculating when is the
 * transition. These are to be handled by the integrator in the interface with
 * the server. Reason being that these elements are server defined and
 * non-standard
 *
 * All mapping is stored as model entity. The only non-model entity stored is
 * base link entity
 *
 * Models that require update should include <physics_engine_interface> tag.
 * Those without the tag are assumed to be static objects.
 *
 * In the <physics_engine_interface> tag, user is supposed to define all the
 * servers used by vessel in different domains if needed.
 * E.g. if the vessel is a submarine, only the underwater and surface definition
 * is needed.
 * User needs to define <init_state> to state the initial server to use.
 *
 * Plugin format:
 *
 * <plugin filename="physics_interface_plugin"
 * name="lotusim::gazebo::PhysicsInterfacePlugin">
 * </plugin>
 *
 * <model name="drone">
 *  <lotus_param>
 *   <physics_engine_interface>
 *    <aerial>
 *      <connection_type>XDynWebSocket</connection_type>
 *      <uri>127.0.0.1:1234</uri>
 *      <thrusters>
 *        <thruster1>propeller1</thruster1>
 *      </thrusters>
 *    </aerial>
 *    <surface>
 *      <connection_type>XDynGRPC</connection_type>
 *      <uri>127.0.0.1:1235</uri>
 *    </surface>
 *    <underwater>
 *      <connection_type>Manual</connection_type>
 *      <uri>127.0.0.1:1236</uri>
 *    </underwater>
 *    <init_state>Surface</init_state>
 *   </physics_engine_interface>
 *  </lotus_param>
 * </model>
 */

class PhysicsInterfacePlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemUpdate {
public:
    PhysicsInterfacePlugin();
    ~PhysicsInterfacePlugin() override;

    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief Create a new Domain interface
     *
     * @param entity
     * @param vessel_name
     * @param physics_sdf
     * @param domain
     * @param interface_map
     */
    void createDomainConnection(
        const gz::sim::Entity& entity,
        const std::string& vessel_name,
        sdf::ElementPtr physics_sdf,
        const lotusim::gazebo::DomainType& domain,
        std::unordered_map<
            gz::sim::Entity,
            std::shared_ptr<PhysicsInterfaceBase>>& interface_map);

    /**
     * @brief Create a physics interface. Users are to edit this function
     * based on the interface created
     *
     * @param entity Entity ID
     * @param name Vessel name
     * @param protocol_type Connection protocol type
     * @param sdf SDF configuration element
     * @return std::shared_ptr<PhysicsInterfaceBase>
     */
    std::shared_ptr<PhysicsInterfaceBase> createConnection(
        const gz::sim::Entity& entity,
        const std::string& name,
        const ConnectionType& protocol_type,
        const sdf::ElementPtr sdf);

    /**
     * @brief Update vessel physics state
     *
     * @param vessel_entity Vessel entity ID
     * @param _info Update info
     * @param _ecm Entity component manager
     */
    void updateVesselState(
        const gz::sim::Entity& vessel_entity,
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm);

    /**
     * @brief When vessel transits between domain and requires to switch server
     *
     * @param _vessel Model entity in gz
     * @param _new_mode The domain vessel is switching to
     * @return true if transition successful
     * @return false otherwise
     */
    bool vesselTransition(gz::sim::Entity _vessel, DomainType _new_mode);

    /**
     * @brief Handle vessel creation
     *
     * @param _entity Entity ID
     * @param _model Model SDF component
     * @param _ecm Entity component manager
     * @return true if loading successful
     * @return false otherwise
     */
    bool loadVessel(
        const gz::sim::Entity& _entity,
        const gz::sim::components::ModelSdf* _model,
        gz::sim::EntityComponentManager* _ecm);

    /**'
     * @brief Handle vessel deletion
     *
     * @param _entity Entity ID
     * @param _model Model SDF component
     * @param _ecm Entity component manager
     * @return true if deletion successful
     * @return false otherwise
     */
    bool deleteVessel(
        const gz::sim::Entity& _entity,
        const gz::sim::components::ModelSdf* _model,
        gz::sim::EntityComponentManager* _ecm);

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief World name
     *
     */
    std::string m_world_name;

    /**
     * @brief ROS node
     *
     */
    rclcpp::Node::SharedPtr m_ros_node;

    /**
     * @brief Stores the PhysicsInterfacePlugin entity
     *
     */
    gz::sim::Entity m_entity;

    /**
     * @brief Shared mutex for all mapping
     *
     */
    mutable std::shared_mutex m_mutex;

    /**
     * @brief List of the vessel entities
     *
     */
    std::vector<gz::sim::Entity> m_vessels_entities;

    /**
     * @brief Mapping for vessel name to base_link entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_base_link_map;

    /**
     * @brief Mapping for vessel name to the model entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_model_map;

    /**
     * @brief Mapping for model entity to vessel name
     *
     */
    std::unordered_map<gz::sim::Entity, std::string> m_vessels_name_map;

    /**
     * @brief Subscription for vessel command array
     *
     */
    rclcpp::Subscription<lotusim_msgs::msg::VesselCmdArray>::SharedPtr
        m_cmd_array_sub;

    /**
     * @brief Shared map of vessel entity to command string
     *
     */
    std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>>
        m_vessels_cmd_map_ptr;

    /**
     * @brief Current vessel interface connected
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<PhysicsInterfaceBase>>
        m_current_vessel_interface;

    /**
     * @brief Current domain the vehicle is in
     *
     */
    std::unordered_map<gz::sim::Entity, DomainType> m_vehicle_current_mode;

    /**
     * @brief Mapping for user set aerial server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<PhysicsInterfaceBase>>
        m_aerial_interface;

    /**
     * @brief Mapping for user set surface server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<PhysicsInterfaceBase>>
        m_surface_interface;

    /**
     * @brief Mapping for user set underwater server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<PhysicsInterfaceBase>>
        m_underwater_interface;
};

}  // namespace lotusim::gazebo
#endif