#ifndef __GZ_ENTITY_MANAGEMENT_PLUGIN_HH__
#define __GZ_ENTITY_MANAGEMENT_PLUGIN_HH__

#include <cstdlib>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/World.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
 * @brief EntityManager
 * EntityManager is the core of Multi-Agent System and is responsible for
 * managing assets, publishing asset pose and handling user cmds.
 */
class EntityManager : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate {
public:
    EntityManager();

    ~EntityManager();

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    // Runtime change
    // void changeRuntime();
    // void changeTimeMultipler();

protected:
    /**
     * @brief Custom user configuration
     *
     * @param _sdf sdf to the plugin. Users are to extract their custom param
     */
    virtual void customUserConfiguration(
        const std::shared_ptr<const sdf::Element> &_sdf);

    /**
     * @brief Custom User update command called after MAS cmd is called and
     * before physics update is done
     */
    virtual void customUserPreUpdate();

    /**
     * @brief Custom User update command called after physics update is done
     * and pose publishedconst std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mfm_msgs::srv::GetTaskForecast::Request> request,
    const std::shared_ptr<mfm_msgs::srv::GetTaskForecast::Response> response
     */
    virtual void customUserPostUpdate();

    /**
     * @brief Custom User add entity command called after built-in addEntity
     * is done
     */
    virtual void customUserAddEntity(const lotusim_msgs::msg::MASCmd &msg);

    /**
     * @brief Custom User delete command called after built-in deleteEntity
     * is done
     */
    virtual void customUserDeleteEntity(const lotusim_msgs::msg::MASCmd &msg);

private:
    /**
     * @brief Method to create entity in the simulation
     * In the msg, only a clean sdf_string is expected. No other changes
     * will be done to the sdf created.
     *
     * @param msg
     */
    std::optional<std::tuple<uint16_t, std::string>> addEntity(
        const lotusim_msgs::msg::MASCmd &msg);

    /**
     * @brief Move existing entity
     * In the message, the vessel_name or entity number and vessel_position
     * is expected.
     *
     * @param msg
     */
    bool moveEntity(const lotusim_msgs::msg::MASCmd &msg);

    /**
     * @brief Delete entity
     * In the message, vessel_name or entity is expected
     *
     * @param msg
     */
    bool deleteEntity(const lotusim_msgs::msg::MASCmd &msg);

    void publishPose(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm);

private:
    rclcpp_action::GoalResponse handleMASCmdArrayGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const lotusim_msgs::action::MASCmdArray::Goal> goal);
    rclcpp_action::CancelResponse handleMASCmdArrayCancel(
        const std::shared_ptr<GoalHandleMASCmdArray> goal_handle);
    void handleMASCmdArrayAccepted(
        const std::shared_ptr<GoalHandleMASCmdArray> goal_handle);
    rclcpp_action::GoalResponse handleMASCmdGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const lotusim_msgs::action::MASCmd::Goal> goal);
    rclcpp_action::CancelResponse handleMASCmdCancel(
        const std::shared_ptr<GoalHandleMASCmd> goal_handle);
    void handleMASCmdAccepted(
        const std::shared_ptr<GoalHandleMASCmd> goal_handle);

    std::shared_ptr<lotusim_msgs::action::MASCmd::Result> handleMASCmd(
        const lotusim_msgs::msg::MASCmd &msg);
    // void CreateBridge(
    //     const std::string &vessel_name,
    //     std::string direction = "@");

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
    gz::sim::EntityComponentManager *m_ecm;

    std::mutex m_cmds_array_mutex;

    std::vector<std::shared_ptr<GoalHandleMASCmdArray>> m_mas_cmds_array;

    std::mutex m_cmds_mutex;

    std::vector<std::shared_ptr<GoalHandleMASCmd>> m_mas_cmds;

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