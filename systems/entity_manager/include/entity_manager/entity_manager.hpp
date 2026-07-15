/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_ENTITY_MANAGEMENT_PLUGIN_HH_
#define LOTUSIM_ENTITY_MANAGEMENT_PLUGIN_HH_

#include <cstdlib>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/World.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shared_mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_msgs/action/mas_cmd.hpp"
#include "lotusim_msgs/action/mas_cmd_array.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"

using GoalHandleMASCmdArray =
    rclcpp_action::ServerGoalHandle<lotusim_msgs::action::MASCmdArray>;
using GoalHandleMASCmd =
    rclcpp_action::ServerGoalHandle<lotusim_msgs::action::MASCmd>;

namespace lotusim::gazebo {

/**
 * @brief Core manager for the Multi-Agent System.
 *
 * The EntityManager is responsible for managing assets, publishing asset poses,
 * and handling user model commands.
 *
 * Topics:
 * - Publisher: `Poses` (<VesselPositionArray>)
 * - Subscriber: `mas_cmd` (<MASCmd>)
 * - Subscriber: `mas_cmd_array` (<MASCmdArray>)
 */
class EntityManager : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemUpdate,
                      public gz::sim::ISystemPostUpdate {
public:
    EntityManager();

    ~EntityManager();

    /**
     * @brief Inherited method from GZ.
     *
     * Called after system is instantiated and all entities
     * and components are loaded from the corresponding SDF world,
     * and before simulation begins exectution.
     *
     * @param _entity The entity this plugin is attached to.
     * @param _sdf The SDF Element associated with this system plugin.
     * @param _ecm The EntityComponentManager of the given simulation instance.
     * @param _eventMgr The EventManager of the given simulation instance.

     */
    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    /**
     * @brief Function called before physics update
     *
     * Has read-write access to world entities and components.
     *
     * This is where systems specify what should happen at time
     * UpdateInfo::simTime.
     *
     * Can be used to modify state before physics runs, for example to
     * apply control signals or perform network synchronization.
     *
     * @param _info The simulation time reached after the function ends.
     * @param _ecm  The EntityComponentManager of the given simulation instance.
     */
    void PreUpdate(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

    /**
     * @briefFunction called during physics update
     *
     * Has read-write access to world entities and components.
     *
     * @param _info
     * @param _ecm
     */
    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

    /**
     * @brief Function called after physics update.
     *
     * Has read-only access to world entities and components.
     * Captures everything that happened at time UpdateInfo::simTime.
     * Used to read out results at the end of a simulation step, e.g.,
     * for sensor or controller updates.
     *
     * @param _info The simulation time reached after the function ends.
     * @param _ecm  The EntityComponentManager of the given simulation instance.
     */
    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) override;

    // Todo: Changing time setting during runtime
    // void changeRuntime();
    // void changeTimeMultipler();

protected:
    /**
     * @brief Custom user configuration
     *
     * @param _sdf sdf to the plugin. Users are to extract their custom param
     */
    virtual void customUserConfiguration(
        const std::shared_ptr<const sdf::Element>& _sdf);

    /**
     * @brief Custom User update command called after MAS cmd is called and
     * before physics update is done
     */
    virtual void customUserPreUpdate();

    /**
     * @brief Custom User update command called after physics update is done
     * and pose published
     */
    virtual void customUserPostUpdate();

    /**
     * @brief Custom User add entity command called after built-in addEntity
     * is done
     */
    virtual void customUserAddEntity(const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Custom User delete command called after built-in deleteEntity
     * is done
     */
    virtual void customUserDeleteEntity(const lotusim_msgs::msg::MASCmd& msg);

private:
    /**
     * @brief Method to create entity in the simulation
     * In the msg, only a clean sdf_string is expected. No other changes
     * will be done to the sdf created.
     *
     * @param msg
     */
    std::optional<std::tuple<uint16_t, std::string>> addEntity(
        const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Move existing entity
     * In the message, the vessel_name or entity number and vessel_position
     * is expected.
     *
     * @param msg
     */
    bool moveEntity(const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Delete entity
     * In the message, vessel_name or entity is expected
     *
     * @param msg
     */
    bool deleteEntity(const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Publishes pose of all vessels in the system
     *
     * @param _info
     * @param _ecm
     */
    void publishPose(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm);

private:
    // MASCmd Array action server functions
    rclcpp_action::GoalResponse handleMASCmdArrayGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const lotusim_msgs::action::MASCmdArray::Goal> goal);
    rclcpp_action::CancelResponse handleMASCmdArrayCancel(
        const std::shared_ptr<GoalHandleMASCmdArray> goal_handle);
    void handleMASCmdArrayAccepted(
        const std::shared_ptr<GoalHandleMASCmdArray> goal_handle);

    // MASCmd action server functions
    rclcpp_action::GoalResponse handleMASCmdGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const lotusim_msgs::action::MASCmd::Goal> goal);
    rclcpp_action::CancelResponse handleMASCmdCancel(
        const std::shared_ptr<GoalHandleMASCmd> goal_handle);
    void handleMASCmdAccepted(
        const std::shared_ptr<GoalHandleMASCmd> goal_handle);

    /**
     * @brief Main function handle all the MAScmd
     *
     * @param msg
     * @return std::shared_ptr<lotusim_msgs::action::MASCmd::Result>
     */
    std::shared_ptr<lotusim_msgs::action::MASCmd::Result> handleMASCmd(
        const lotusim_msgs::msg::MASCmd& msg);

protected:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /**
     * @brief GZ world entity
     *
     */
    gz::sim::Entity m_world_entity;

    /**
     * @brief Name of world
     *
     */
    std::string m_world_name;

    /**
     * @brief ROS node
     *
     */
    rclcpp::Node::SharedPtr m_ros_node;

    /**
     * @brief GZ Entity Component Manager
     *
     */
    gz::sim::EntityComponentManager* m_ecm;

    // MAS cmds and mutex
    std::mutex m_cmds_array_mutex;
    std::vector<std::shared_ptr<GoalHandleMASCmdArray>> m_mas_cmds_array;
    std::mutex m_cmds_mutex;
    std::vector<std::shared_ptr<GoalHandleMASCmd>> m_mas_cmds;

    mutable std::shared_mutex m_variable_mutex;

    /**
     * @brief Mapping of string to Gz entity of the current vessel in the
     * system
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_entities;

    /**
     * @brief Mapping from entity to name of the current vessel in the
     * system
     *
     */
    std::unordered_map<gz::sim::Entity, std::string> m_vessels_names;

    /**
     * @brief GZ Creator class
     *
     */
    std::unique_ptr<gz::sim::SdfEntityCreator> m_creator;

private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;

    std::shared_ptr<std::thread> m_ros_node_thread;

    std::vector<rclcpp::CallbackGroup::SharedPtr> m_callback_group;

    rclcpp::Publisher<lotusim_msgs::msg::VesselPositionArray>::SharedPtr
        m_pose_pub;

    rclcpp_action::Server<lotusim_msgs::action::MASCmdArray>::SharedPtr
        m_cmd_array_action;

    rclcpp_action::Server<lotusim_msgs::action::MASCmd>::SharedPtr m_cmd_action;
};

}  // namespace lotusim::gazebo
#endif