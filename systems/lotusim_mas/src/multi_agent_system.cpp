/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_mas/multi_agent_system.hpp"

namespace lotusim::gazebo {

MultiAgentSystem::MultiAgentSystem()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}

MultiAgentSystem::~MultiAgentSystem()
{
    shutdown();
    m_logger->info(
        "MultiAgentSystem::~MultiAgentSystem: MultiAgentSystem successfully shutdown.");
}

void MultiAgentSystem::shutdown()
{
    m_running.store(false);
    if (m_executor) {
        m_executor->cancel();
    }

    if (m_ros_node_thread && m_ros_node_thread->joinable()) {
        m_ros_node_thread->join();
    }
    m_executor.reset();
    m_logger->info(
        "MultiAgentSystem::shutdown: MultiAgentSystem shutdown complete");
}

void MultiAgentSystem::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _eventMgr)
{
    m_world_entity = _entity;
    m_world_name = lotusim::common::getWorldName(_ecm);
    m_ecm = &_ecm;
    m_logger = logger::createConsoleAndFileLogger(
        "mas",
        m_world_name + "_mas_plugin.txt");
    if (!MultiAgentSystem::initROSNode()) {
        m_logger->error(
            "MultiAgentSystem::Configure: RCLCPP context shutdown.");
    }

    m_creator = std::make_shared<gz::sim::SdfEntityCreator>(_ecm, _eventMgr);
    m_entity_spawner = std::make_shared<lotusim::mas::EntitySpawner>(
        _ecm,
        m_world_entity,
        m_creator,
        m_logger);
    m_scenario_manager = std::make_unique<lotusim::scenario::ScenarioManager>(
        m_ros_node,
        m_entity_spawner,
        m_logger,
        &_ecm);

    customUserConfiguration(_sdf);
    m_logger->info("MultiAgentSystem::Configure: MAS started.");
}

bool MultiAgentSystem::initROSNode()
{
    if (!rclcpp::ok()) {
        return false;
    }
    m_ros_node = rclcpp::Node::make_shared("mas_node", m_world_name);
    m_pose_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::VesselPositionArray>(
            "poses",
            rclcpp::QoS(10));

    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));
    // Create cmd array action server with default queue depth of 10.
    m_cmd_array_action =
        rclcpp_action::create_server<lotusim_msgs::action::MASCmdArray>(
            m_ros_node,
            "mas_cmd_array",
            std::bind(
                &MultiAgentSystem::handleMASCmdArrayGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &MultiAgentSystem::handleMASCmdArrayCancel,
                this,
                std::placeholders::_1),
            std::bind(
                &MultiAgentSystem::handleMASCmdArrayAccepted,
                this,
                std::placeholders::_1),
            rcl_action_server_get_default_options(),
            m_callback_group.back());

    // Custom QOS for individual MultiAgentSystem cmd. Queue store up to 20.
    const rmw_qos_profile_t custom_profile = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        20,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false};

    auto qos = rcl_action_server_get_default_options();
    qos.goal_service_qos = custom_profile;
    qos.cancel_service_qos = custom_profile;
    qos.result_service_qos = custom_profile;
    qos.feedback_topic_qos = custom_profile;

    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));
    m_cmd_action = rclcpp_action::create_server<lotusim_msgs::action::MASCmd>(
        m_ros_node,
        "mas_cmd",
        std::bind(
            &MultiAgentSystem::handleMASCmdGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &MultiAgentSystem::handleMASCmdCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &MultiAgentSystem::handleMASCmdAccepted,
            this,
            std::placeholders::_1),
        qos,
        m_callback_group.back());

    m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_ros_node);
    m_running.store(true);
    m_ros_node_thread =
        std::make_shared<std::thread>([&]() { m_executor->spin(); });
    m_logger->info(
        "MultiAgentSystem::initROSNode: MultiAgentSystem ROS node initiated");
    return true;
}

void MultiAgentSystem::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    // Handle MultiAgentSystem Array cmd
    {
        std::vector<std::shared_ptr<GoalHandleMASCmdArray>> tmp_vec;
        {
            std::lock_guard<std::mutex> lock(m_cmds_array_mutex);
            tmp_vec = m_mas_cmds_array;
            m_mas_cmds_array.clear();
        }
        lotusim::common::shuffleOrder<std::shared_ptr<GoalHandleMASCmdArray>>(
            tmp_vec,
            lotusim::common::RandomisedType::RANDOM);
        for (auto&& action_handle : tmp_vec) {
            auto result =
                std::make_shared<lotusim_msgs::action::MASCmdArray::Result>();
            for (auto&& cmd : action_handle->get_goal()->cmd) {
                auto res = handleMASCmd(cmd);
                result->result.push_back(res->result);
                result->name.push_back(res->name);
                result->entity.push_back(res->entity);
            }
            try {
                action_handle->succeed(result);
            } catch (std::runtime_error& e) {
                m_logger->error(
                    "MultiAgentSystem::PreUpdate:ERROR Unable to reply action. Action dropped\n{}",
                    e.what());
            } catch (...) {
                m_logger->error(
                    "MultiAgentSystem::PreUpdate:ERROR Unknown error. Unable to reply action. Action dropped");
            }
        }
    }

    // Handle mas cmds
    {
        std::vector<std::shared_ptr<GoalHandleMASCmd>> tmp_vec;
        {
            std::lock_guard<std::mutex> lock(m_cmds_mutex);
            tmp_vec = m_mas_cmds;
            m_mas_cmds.clear();
        }

        lotusim::common::shuffleOrder(
            tmp_vec,
            lotusim::common::RandomisedType::RANDOM);

        for (auto&& action_handle : tmp_vec) {
            auto res = handleMASCmd(action_handle->get_goal()->cmd);
            try {
                action_handle->succeed(res);
            } catch (std::runtime_error& e) {
                m_logger->error(
                    "MultiAgentSystem::PreUpdate:ERROR Unable to reply action. Action dropped\n{}",
                    e.what());
            } catch (...) {
                m_logger->error(
                    "MultiAgentSystem::PreUpdate:ERROR Unknown error. Unable to reply action. Action dropped");
            }
        }
    }
    customUserPreUpdate();
}

std::shared_ptr<lotusim_msgs::action::MASCmd::Result>
MultiAgentSystem::handleMASCmd(const lotusim_msgs::msg::MASCmd& cmd)
{
    auto result = std::make_shared<lotusim_msgs::action::MASCmd::Result>();
    try {
        result->result = false;
        switch (cmd.cmd_type) {
            case lotusim_msgs::msg::MASCmd::CREATE_CMD: {
                auto vessel_name = cmd.vessel_name;
                std::optional<std::tuple<uint16_t, std::string>> res =
                    std::nullopt;

                if (isValidRosName(vessel_name)) {
                    res = m_entity_spawner->addEntity(cmd);
                } else {
                    m_logger->error(
                        "MultiAgentSystem::handleMASCmd: Invalid vessel name '{}': names must not start with a number or contain invalid characters. Skipping.",
                        vessel_name);
                }

                if (res) {
                    auto [entity, vessel_name] = *res;
                    customUserAddEntity(cmd);
                    result->result = true;
                    result->name = vessel_name;
                    result->entity = entity;
                } else {
                    result->result = false;
                    result->name = "invalid_vessel";
                    result->entity = 0;
                }
                break;
            }
            case lotusim_msgs::msg::MASCmd::DELETE_CMD: {
                result->result = m_entity_spawner->deleteEntity(cmd);
                result->name = cmd.vessel_name;
                result->entity = cmd.entity;
                if (result->result) {
                    customUserDeleteEntity(cmd);
                }
                break;
            }
            case lotusim_msgs::msg::MASCmd::MOVE_CMD: {
                result->result = m_entity_spawner->moveEntity(cmd);
                result->name = cmd.vessel_name;
                result->entity = cmd.entity;
                break;
            }
            default: {
                m_logger->error(
                    "MultiAgentSystem::PreUpdate: MASCmd without proper type. Vessel: {}, Entity: {}",
                    cmd.vessel_name,
                    cmd.entity);
                result->name = "error_cmd";
                result->entity = 0;
                break;
            }
        }
    } catch (std::runtime_error& e) {
        m_logger->error(
            "MultiAgentSystem::PreUpdate:ERROR Vessel: {}, Entity: {}, \n{}",
            cmd.vessel_name,
            cmd.entity,
            e.what());
        result->name = "error_cmd";
        result->entity = 0;
    } catch (...) {
        m_logger->error(
            "MultiAgentSystem::PreUpdate:ERROR Vessel: {}, Entity: {}, unknown error",
            cmd.vessel_name,
            cmd.entity);
        result->name = "error_cmd";
        result->entity = 0;
    }
    return result;
}

void MultiAgentSystem::Update(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager& _ecm)
{
    // Seperating MultiAgentSystem cmd handle and other plugins as
    // MultiAgentSystem cmd may fail to run
    _ecm.EachNew<gz::sim::components::
                     ModelSdf>([this](
                                   const gz::sim::Entity& _entity,
                                   const gz::sim::components::ModelSdf*) {
        auto name_opt = m_ecm->Component<gz::sim::components::Name>(_entity);
        std::string vessel_name;
        if (name_opt) {
            vessel_name = name_opt->Data();
            m_logger->info(
                "MultiAgentSystem::EachNew: Vessel {}, {} spawned.",
                _entity,
                vessel_name);
        } else {
            m_logger->warn(
                "MultiAgentSystem::EachNew: New vessel with no name found. Ignoring model.");
            return true;
        }

        m_entity_spawner->registerNewEntity(_entity, vessel_name);
        return true;
    });

    _ecm.EachRemoved<gz::sim::components::ModelSdf>(
        [this](
            const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf*) {
            auto name_opt =
                m_ecm->Component<gz::sim::components::Name>(_entity);
            if (!name_opt) {
                return true;
            }
            m_entity_spawner->unregisterEntity(_entity);
            return true;
        });
}
void MultiAgentSystem::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    publishPose(_info, _ecm);
    customUserPostUpdate();
}

rclcpp_action::GoalResponse MultiAgentSystem::handleMASCmdArrayGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const lotusim_msgs::action::MASCmdArray::Goal>)
{
    m_logger->info(
        "MultiAgentSystem::handleMASCmdArrayGoal: Received MASCmdArray.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiAgentSystem::handleMASCmdArrayCancel(
    const std::shared_ptr<GoalHandleMASCmdArray>)
{
    // Not allowed to cancel for now
    return rclcpp_action::CancelResponse::REJECT;
}

void MultiAgentSystem::handleMASCmdArrayAccepted(
    const std::shared_ptr<GoalHandleMASCmdArray> goal_handle)
{
    std::lock_guard<std::mutex> lock(m_cmds_array_mutex);
    m_mas_cmds_array.push_back(goal_handle);
}

rclcpp_action::GoalResponse MultiAgentSystem::handleMASCmdGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const lotusim_msgs::action::MASCmd::Goal>)
{
    m_logger->info("MultiAgentSystem::handleMASCmdGoal: Received MASCmd.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiAgentSystem::handleMASCmdCancel(
    const std::shared_ptr<GoalHandleMASCmd>)
{
    // Not allowed to cancel for now
    return rclcpp_action::CancelResponse::REJECT;
}

void MultiAgentSystem::handleMASCmdAccepted(
    const std::shared_ptr<GoalHandleMASCmd> goal_handle)
{
    std::lock_guard<std::mutex> lock(m_cmds_mutex);
    m_mas_cmds.push_back(goal_handle);
}

void MultiAgentSystem::customUserConfiguration(
    const std::shared_ptr<const sdf::Element>&)
{
    return;
}

void MultiAgentSystem::customUserPreUpdate()
{
    return;
}

void MultiAgentSystem::customUserPostUpdate()
{
    return;
}

void MultiAgentSystem::customUserAddEntity(const lotusim_msgs::msg::MASCmd&)
{
    return;
}

void MultiAgentSystem::customUserDeleteEntity(const lotusim_msgs::msg::MASCmd&)
{
    return;
}

void MultiAgentSystem::publishPose(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    lotusim_msgs::msg::VesselPositionArray array_msg;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime)
            .count();
    array_msg.header.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    array_msg.header.stamp.nanosec =
        static_cast<uint32_t>(simTimeNs % 1000000000);
    array_msg.header.frame_id = "world";

    auto lock = m_entity_spawner->sharedLock();
    for (auto& [entity, name] : m_entity_spawner->vesselNames()) {
        auto pose = worldPose(entity, _ecm);
        auto latLonEle =
            lotusim::common::XYToLatLong(_ecm, pose.X(), pose.Y(), pose.Z());
        if (!latLonEle) {
            m_logger->warn(
                "MultiAgentSystem::PostUpdate: Vessel: {} unable to find coordinate. Skip pose update.",
                name);
            continue;
        }

        lotusim_msgs::msg::VesselPosition msg;
        msg.vessel_name = name;

        msg.pose.position.x = pose.X();
        msg.pose.position.y = pose.Y();
        msg.pose.position.z = pose.Z();
        msg.pose.orientation.w = pose.Rot().W();
        msg.pose.orientation.x = pose.Rot().X();
        msg.pose.orientation.y = pose.Rot().Y();
        msg.pose.orientation.z = pose.Rot().Z();

        msg.geo_point.latitude = std::get<0>(latLonEle.value());
        msg.geo_point.longitude = std::get<1>(latLonEle.value());
        msg.geo_point.altitude = std::get<2>(latLonEle.value());

        array_msg.vessels.push_back(msg);
    }
    m_pose_pub->publish(array_msg);
}

bool MultiAgentSystem::isValidRosName(const std::string& name)
{
    if (name.empty())
        return false;
    if (std::isdigit(static_cast<unsigned char>(name.front())))
        return false;
    for (char c : name) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            return false;
        }
    }
    return true;
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::MultiAgentSystem,
    gz::sim::System,
    lotusim::gazebo::MultiAgentSystem::ISystemConfigure,
    lotusim::gazebo::MultiAgentSystem::ISystemPreUpdate,
    lotusim::gazebo::MultiAgentSystem::ISystemUpdate,
    lotusim::gazebo::MultiAgentSystem::ISystemPostUpdate)