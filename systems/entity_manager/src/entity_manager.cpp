/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "entity_manager/entity_manager.hpp"

namespace lotusim::gazebo {

EntityManager::EntityManager()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}

EntityManager::~EntityManager()
{
    rclcpp::shutdown();
    m_ros_node_thread->join();
    m_logger->info(
        "EntityManager::~EntityManager: EntityManager successfully shutdown.");
}

void EntityManager::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _eventMgr)
{
    m_world_entity = _entity;
    m_world_name = lotusim::common::getWorldName(_ecm);
    m_ecm = &_ecm;
    m_creator = std::make_unique<gz::sim::SdfEntityCreator>(_ecm, _eventMgr);

    m_logger = logger::createConsoleAndFileLogger(
        "entity_managerment",
        m_world_name + "_entity_managerment_plugin.txt");
    m_ros_node =
        rclcpp::Node::make_shared("gz_entity_management_node", m_world_name);

    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));

    m_pose_pub =
        m_ros_node->create_publisher<lotusim_msgs::msg::VesselPositionArray>(
            "poses",
            rclcpp::QoS(10));

    // Create cmd array action server with default queue depth of 10.
    m_callback_group.push_back(m_ros_node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive));
    m_cmd_array_action =
        rclcpp_action::create_server<lotusim_msgs::action::MASCmdArray>(
            m_ros_node,
            "mas_cmd_array",
            std::bind(
                &EntityManager::handleMASCmdArrayGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &EntityManager::handleMASCmdArrayCancel,
                this,
                std::placeholders::_1),
            std::bind(
                &EntityManager::handleMASCmdArrayAccepted,
                this,
                std::placeholders::_1),
            rcl_action_server_get_default_options(),
            m_callback_group.back());

    // Custom QOS for individual MAS cmd. Queue store up to 20.
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
            &EntityManager::handleMASCmdGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &EntityManager::handleMASCmdCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &EntityManager::handleMASCmdAccepted,
            this,
            std::placeholders::_1),
        qos,
        m_callback_group.back());

    if (rclcpp::ok()) {
        m_executor =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_ros_node);
        m_ros_node_thread =
            std::make_shared<std::thread>([&]() { m_executor->spin(); });
    } else {
        m_logger->error("EntityManager::Configure: RCLCPP context shutdown.");
    }
    m_logger->info("EntityManager::Configure: Entity Manager started.");
    customUserConfiguration(_sdf);
}

void EntityManager::PreUpdate(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager&)
{
    // Handle MAS Array cmd
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
                    "EntityManager::PreUpdate:ERROR Unable to reply action. Action dropped\n{}",
                    e.what());
            } catch (...) {
                m_logger->error(
                    "EntityManager::PreUpdate:ERROR Unknown error. Unable to reply action. Action dropped");
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
                    "EntityManager::PreUpdate:ERROR Unable to reply action. Action dropped\n{}",
                    e.what());
            } catch (...) {
                m_logger->error(
                    "EntityManager::PreUpdate:ERROR Unknown error. Unable to reply action. Action dropped");
            }
        }
    }
    customUserPreUpdate();
}

std::shared_ptr<lotusim_msgs::action::MASCmd::Result>
EntityManager::handleMASCmd(const lotusim_msgs::msg::MASCmd& cmd)
{
    auto result = std::make_shared<lotusim_msgs::action::MASCmd::Result>();
    try {
        result->result = false;
        switch (cmd.cmd_type) {
            case lotusim_msgs::msg::MASCmd::CREATE_CMD: {
                auto res = addEntity(cmd);
                if (res) {
                    auto [entity, vessel_name] = *res;
                    customUserAddEntity(cmd);
                    result->result = true;
                    result->name = vessel_name;
                    result->entity = entity;
                }
                break;
            }
            case lotusim_msgs::msg::MASCmd::DELETE_CMD: {
                result->result = deleteEntity(cmd);
                result->name = cmd.vessel_name;
                result->entity = cmd.entity;
                if (result->result) {
                    customUserDeleteEntity(cmd);
                }
                break;
            }
            case lotusim_msgs::msg::MASCmd::MOVE_CMD: {
                result->result = moveEntity(cmd);
                result->name = cmd.vessel_name;
                result->entity = cmd.entity;
                break;
            }
            default: {
                m_logger->error(
                    "EntityManager::PreUpdate: MASCmd without proper type. Vessel: {}, Entity: {}",
                    cmd.vessel_name,
                    cmd.entity);
                result->name = "error_cmd";
                result->entity = 0;
                break;
            }
        }
    } catch (std::runtime_error &e) {
        m_logger->error(
            "EntityManager::PreUpdate:ERROR Vessel: {}, Entity: {}, \n{}",
            cmd.vessel_name,
            cmd.entity,
            e.what());
        result->name = "error_cmd";
        result->entity = 0;
    } catch (...) {
        m_logger->error(
            "EntityManager::PreUpdate:ERROR Vessel: {}, Entity: {}, unknown error",
            cmd.vessel_name,
            cmd.entity);
        result->name = "error_cmd";
        result->entity = 0;
    }
    return result;
}

void EntityManager::Update(
    const gz::sim::UpdateInfo&,
    gz::sim::EntityComponentManager& _ecm)
{
    // Seperating MAS cmd handle and other plugins as MAS cmd may fail to run
    _ecm.EachNew<gz::sim::components::
                     ModelSdf>([this](
                                   const gz::sim::Entity& _entity,
                                   const gz::sim::components::ModelSdf*) {
        auto name_opt = m_ecm->Component<gz::sim::components::Name>(_entity);
        std::string vessel_name;
        if (name_opt) {
            vessel_name = name_opt->Data();
            m_logger->info(
                "EntityManager::EachNew: Vessel {}, {} spawned.",
                _entity,
                vessel_name);
        } else {
            m_logger->warn(
                "EntityManager::EachNew: New vessel with no name found. Ignoring model.");
            return true;
        }

        {
            std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
            m_vessels_entities[vessel_name] = _entity;
            m_vessels_names[_entity] = vessel_name;
        }

        // Making the base_link in the model report velocity
        auto child_link =
            m_ecm->ChildrenByComponents(_entity, gz::sim::components::Link());
        for (auto&& link : child_link) {
            auto name_opt = m_ecm->Component<gz::sim::components::Name>(link);
            if (name_opt &&
                name_opt->Data().find("base_link") != std::string::npos) {
                auto base_link = link;
                gz::sim::Link _link(base_link);
                _link.EnableVelocityChecks(*m_ecm);
                break;
            }
        }
        return true;
    });

    _ecm.EachRemoved<gz::sim::components::ModelSdf>(
        [this](
            const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf*) {
            auto name_opt =
                m_ecm->Component<gz::sim::components::Name>(_entity);
            std::string vessel_name;
            if (!name_opt) {
                return true;
            }
            std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
            m_vessels_entities.erase(vessel_name);
            m_vessels_names.erase(_entity);
            return true;
        });
}
void EntityManager::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    publishPose(_info, _ecm);
    customUserPostUpdate();
}

rclcpp_action::GoalResponse EntityManager::handleMASCmdArrayGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const lotusim_msgs::action::MASCmdArray::Goal>)
{
    m_logger->info(
        "EntityManager::handleMASCmdArrayGoal: Received MASCmdArray.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EntityManager::handleMASCmdArrayCancel(
    const std::shared_ptr<GoalHandleMASCmdArray>)
{
    // Not allowed to cancel for now
    return rclcpp_action::CancelResponse::REJECT;
}

void EntityManager::handleMASCmdArrayAccepted(
    const std::shared_ptr<GoalHandleMASCmdArray> goal_handle)
{
    std::lock_guard<std::mutex> lock(m_cmds_array_mutex);
    m_mas_cmds_array.push_back(goal_handle);
}

rclcpp_action::GoalResponse EntityManager::handleMASCmdGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const lotusim_msgs::action::MASCmd::Goal>)
{
    m_logger->info("EntityManager::handleMASCmdGoal: Received MASCmd.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EntityManager::handleMASCmdCancel(
    const std::shared_ptr<GoalHandleMASCmd>)
{
    // Not allowed to cancel for now
    return rclcpp_action::CancelResponse::REJECT;
}

void EntityManager::handleMASCmdAccepted(
    const std::shared_ptr<GoalHandleMASCmd> goal_handle)
{
    std::lock_guard<std::mutex> lock(m_cmds_mutex);
    m_mas_cmds.push_back(goal_handle);
}

void EntityManager::customUserConfiguration(
    const std::shared_ptr<const sdf::Element>&)
{
    return;
}

void EntityManager::customUserPreUpdate()
{
    return;
}

void EntityManager::customUserPostUpdate()
{
    return;
}

void EntityManager::customUserAddEntity(const lotusim_msgs::msg::MASCmd&)
{
    return;
}

void EntityManager::customUserDeleteEntity(const lotusim_msgs::msg::MASCmd&)
{
    return;
}

std::optional<std::tuple<uint16_t, std::string>> EntityManager::addEntity(
    const lotusim_msgs::msg::MASCmd& msg)
{
    sdf::Root root;
    sdf::Errors errors;
    tinyxml2::XMLDocument lotus_param_doc;

    // If model name is given, we will load the asset based on the name and the
    // lotus param is expected in the sdf string Else we will expect the whole
    // model sdf to be in sdf string
    if (msg.model_name.empty()) {
        errors = root.LoadSdfString(msg.sdf_string);
    } else {
        // Combining the lotus param into the sdf
        const char* asset_path = std::getenv("LOTUSIM_MODELS_PATH");
        if (!asset_path) {
            m_logger->error(
                "EntityManager::addEntity: Environment variable LOTUSIM_MODELS_PATH is not set. Please set and restart lotusim.");
            return std::nullopt;
        }

        std::string file_path = std::string(asset_path) + "/" + msg.model_name;

        if (!msg.sdf_string.empty()) {
            tinyxml2::XMLDocument sdfDoc;
            tinyxml2::XMLError loadResult =
                sdfDoc.LoadFile((file_path + "/model.sdf").c_str());
            if (loadResult != tinyxml2::XML_SUCCESS) {
                m_logger->error("    file.{}", file_path + "/model.sdf");
                return std::nullopt;
            }

            tinyxml2::XMLElement* sdfElem = sdfDoc.FirstChildElement("sdf");
            if (!sdfElem) {
                m_logger->error(
                    "EntityManager::addEntity: No <sdf> element found.");
                return std::nullopt;
            }

            tinyxml2::XMLElement* modelElem =
                sdfElem->FirstChildElement("model");
            if (!modelElem) {
                m_logger->error(
                    "EntityManager::addEntity: No <model> element found.");
                return std::nullopt;
            }
            tinyxml2::XMLDocument lotusDoc;
            lotusDoc.Parse(msg.sdf_string.c_str());

            tinyxml2::XMLElement* lotusElem =
                lotusDoc.FirstChildElement("lotus_param");
            if (!lotusElem) {
                m_logger->error(
                    "EntityManager::addEntity: Could not parse lotus_param.");
                return std::nullopt;
            }

            tinyxml2::XMLElement* lotusClone =
                static_cast<tinyxml2::XMLElement*>(
                    lotusElem->DeepClone(&sdfDoc));
            modelElem->InsertEndChild(lotusClone);

            tinyxml2::XMLPrinter printer;
            sdfDoc.Print(&printer);
            std::string modifiedSDF = printer.CStr();
            m_logger->debug(
                "EntityManager::addEntity: modified loaded sdf.\n{}",
                modifiedSDF);

            sdf::Errors errors = root.LoadSdfString(modifiedSDF);
            if (!errors.empty()) {
                m_logger->error(
                    "EntityManager::addEntity: Errors when loading modified SDF into sdf::Root:");
                for (const auto& err : errors)
                    m_logger->error(err.Message());
                return std::nullopt;
            }
        }
    }

    if (!errors.empty()) {
        for (auto&& err : errors) {
            m_logger->error("EntityManager::addEntity: {}", err.Message());
        }
        return std::nullopt;
    }

    if (!root.Model()) {
        m_logger->error("EntityManager::addEntity: sdf loaded but no model");
        return std::nullopt;
    }

    sdf::Model model = *root.Model();
    std::string desiredName = model.Name();
    if (!msg.vessel_name.empty()) {
        desiredName = msg.vessel_name;
    }

    // Check no conflicting name
    if (!desiredName.empty() &&
        gz::sim::kNullEntity !=
            m_ecm->EntityByComponents(
                gz::sim::components::Name(desiredName),
                gz::sim::components::ParentEntity(m_world_entity))) {
        // Generate unique name
        std::string newName = desiredName;
        int i = 0;
        while (gz::sim::kNullEntity !=
               m_ecm->EntityByComponents(
                   gz::sim::components::Name(newName),
                   gz::sim::components::ParentEntity(m_world_entity))) {
            newName = desiredName + "_" + std::to_string(i++);
        }
        desiredName = newName;
    }
    model.SetName(desiredName);

    // Creating the entity
    gz::sim::Entity entity{gz::sim::kNullEntity};
    entity = m_creator->CreateEntities(&model);
    if (entity == gz::sim::kNullEntity) {
        m_logger->error(
            "EntityManager::addEntity: Failed to create named [{}]",
            desiredName);
        return std::make_tuple(entity, "");
    }
    // Set parent
    m_creator->SetParent(entity, m_world_entity);
    m_logger->info(
        "EntityManager::addEntity: Created entity [{}] named [{}]",
        entity,
        desiredName);

    // Change to requested position
    lotusim_msgs::msg::MASCmd new_msg = msg;
    new_msg.entity = entity;
    new_msg.cmd_type = lotusim_msgs::msg::MASCmd::MOVE_CMD;
    moveEntity(new_msg);
    return std::make_tuple(entity, desiredName);
}

bool EntityManager::moveEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    try {
        gz::sim::Entity vessel_entity;
        if (msg.entity) {
            vessel_entity = msg.entity;
        } else if (
            !msg.vessel_name.empty() &&
            m_vessels_entities.find(msg.vessel_name) !=
                m_vessels_entities.end()) {
            std::shared_lock<std::shared_mutex> lock(m_variable_mutex);
            vessel_entity = m_vessels_entities.at(msg.vessel_name);
        } else {
            m_logger->error(
                "EntityManager::moveEntity: Called without providing entity or vessel_name");
            return false;
        }

        gz::math::Pose3 pose = gz::math::Pose3<double>(
            msg.vessel_position.position.x,
            msg.vessel_position.position.y,
            msg.vessel_position.position.z,
            msg.vessel_position.orientation.w,
            msg.vessel_position.orientation.x,
            msg.vessel_position.orientation.y,
            msg.vessel_position.orientation.z);

        if (msg.geo_point.latitude != 0 || msg.geo_point.longitude != 0) {
            auto xy_opt = lotusim::common::XYFromLatLong(
                *m_ecm,
                msg.geo_point.latitude,
                msg.geo_point.longitude);

            if (xy_opt) {
                pose.Pos().X() = std::get<0>(xy_opt.value());
                pose.Pos().Y() = std::get<1>(xy_opt.value());
            }
        }

        bool res = m_ecm->SetComponentData<gz::sim::components::Pose>(
            vessel_entity,
            pose);
        if (!res) {
            m_logger->warn(
                "EntityManager::moveEntity: Failed to set componenent data {}, entity: {}",
                msg.vessel_name,
                msg.entity);
        }

        m_ecm->SetChanged(
            vessel_entity,
            gz::sim::components::Pose::typeId,
            gz::sim::ComponentState::OneTimeChange);

    } catch (std::out_of_range const& exc) {
        m_logger->error(
            "EntityManager::moveEntity: Failed to move {}, entity: {}",
            msg.vessel_name,
            msg.entity);

    } catch (...) {
        m_logger->error(
            "EntityManager::moveEntity: Failed to move {}, entity: {}",
            msg.vessel_name,
            msg.entity);
        return false;
    }
    return true;
}

bool EntityManager::deleteEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    gz::sim::Entity vessel_entity;
    try {
        std::shared_lock<std::shared_mutex> lock(m_variable_mutex);

        if (msg.entity) {
            vessel_entity = msg.entity;
        } else if (
            !msg.vessel_name.empty() &&
            m_vessels_entities.find(msg.vessel_name) !=
                m_vessels_entities.end()) {
            vessel_entity = m_vessels_entities[msg.vessel_name];
        } else {
            m_logger->error(
                "EntityManager::deleteEntity: Called without providing entity or vessel_name");
            return false;
        }
    } catch (...) {
        m_logger->error(
            "EntityManager::deleteEntity: Failed to move {}, entity: {}",
            msg.vessel_name,
            msg.entity);
        return false;
    }
    m_creator->RequestRemoveEntity(vessel_entity);
    return true;
}

void EntityManager::publishPose(
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
    std::shared_lock<std::shared_mutex> lock(m_variable_mutex);
    for (auto&& vessel_map : m_vessels_names) {
        auto entity = vessel_map.first;
        auto latLonEle = sphericalCoordinates(entity, _ecm);
        auto pose = worldPose(entity, _ecm);
        if (!latLonEle) {
            m_logger->warn(
                "EntityManager::PostUpdate: Vessel: {} unable to find coordinate. Skip pose update.",
                vessel_map.second);
        }

        lotusim_msgs::msg::VesselPosition msg;
        msg.vessel_name = vessel_map.second;

        msg.pose.position.x = pose.X();
        msg.pose.position.y = pose.Y();
        msg.pose.position.z = pose.Z();
        msg.pose.orientation.w = pose.Rot().W();
        msg.pose.orientation.x = pose.Rot().X();
        msg.pose.orientation.y = pose.Rot().Y();
        msg.pose.orientation.z = pose.Rot().Z();

        msg.geo_point.latitude = latLonEle.value().X();
        msg.geo_point.longitude = latLonEle.value().Y();
        msg.geo_point.altitude = latLonEle.value().Z();

        array_msg.vessels.push_back(msg);
    }
    m_pose_pub->publish(array_msg);
}
}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::EntityManager,
    gz::sim::System,
    lotusim::gazebo::EntityManager::ISystemConfigure,
    lotusim::gazebo::EntityManager::ISystemPreUpdate,
    lotusim::gazebo::EntityManager::ISystemUpdate,
    lotusim::gazebo::EntityManager::ISystemPostUpdate)