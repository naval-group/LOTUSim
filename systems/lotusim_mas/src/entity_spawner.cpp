/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_mas/entity_spawner.hpp"

#include <tinyxml2.h>

#include <cstdlib>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Root.hh>

#include "lotusim_common/common.hpp"

namespace lotusim::mas {

EntitySpawner::EntitySpawner(
    gz::sim::EntityComponentManager& ecm,
    gz::sim::Entity world_entity,
    std::shared_ptr<gz::sim::SdfEntityCreator> creator,
    std::shared_ptr<spdlog::logger> logger)
    : m_logger{std::move(logger)}
    , m_ecm{ecm}
    , m_world_entity{world_entity}
    , m_creator{creator}
{
}

std::optional<std::tuple<uint16_t, std::string>> EntitySpawner::addEntity(
    const lotusim_msgs::msg::MASCmd& msg)
{
    sdf::Root root;
    sdf::Errors errors;

    if (msg.model_name.empty()) {
        // Full SDF provided directly in sdf_string.
        errors = root.LoadSdfString(msg.sdf_string);
    } else {
        // Load model from disk; optionally inject a <lotus_param> block.
        const char* asset_path = std::getenv("LOTUSIM_MODELS_PATH");
        if (!asset_path) {
            m_logger->error(
                "EntitySpawner::addEntity: LOTUSIM_MODELS_PATH is not set.");
            return std::nullopt;
        }

        const std::string file_path =
            std::string(asset_path) + "/" + msg.model_name + "/model.sdf";

        if (msg.sdf_string.empty()) {
            // No lotus_param — load file as-is.
            errors = root.Load(file_path);
        } else {
            // Inject the <lotus_param> block into the loaded SDF.
            tinyxml2::XMLDocument sdf_doc;
            if (sdf_doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
                m_logger->error(
                    "EntitySpawner::addEntity: Failed to load SDF file '{}'",
                    file_path);
                return std::nullopt;
            }

            tinyxml2::XMLElement* sdf_elem = sdf_doc.FirstChildElement("sdf");
            if (!sdf_elem) {
                m_logger->error(
                    "EntitySpawner::addEntity: No <sdf> element in '{}'",
                    file_path);
                return std::nullopt;
            }

            tinyxml2::XMLElement* model_elem =
                sdf_elem->FirstChildElement("model");
            if (!model_elem) {
                m_logger->error(
                    "EntitySpawner::addEntity: No <model> element in '{}'",
                    file_path);
                return std::nullopt;
            }

            tinyxml2::XMLDocument lotus_doc;
            lotus_doc.Parse(msg.sdf_string.c_str());
            tinyxml2::XMLElement* lotus_elem =
                lotus_doc.FirstChildElement("lotus_param");
            if (!lotus_elem) {
                m_logger->error(
                    "EntitySpawner::addEntity: Could not parse <lotus_param> "
                    "from sdf_string.");
                return std::nullopt;
            }

            model_elem->InsertEndChild(
                static_cast<tinyxml2::XMLElement*>(
                    lotus_elem->DeepClone(&sdf_doc)));

            tinyxml2::XMLPrinter printer;
            sdf_doc.Print(&printer);
            const std::string modified_sdf = printer.CStr();

            m_logger->debug(
                "EntitySpawner::addEntity: Modified SDF:\n{}",
                modified_sdf);

            errors = root.LoadSdfString(modified_sdf);
            if (!errors.empty()) {
                for (const auto& err : errors)
                    m_logger->error(
                        "EntitySpawner::addEntity: SDF error: {}",
                        err.Message());
                return std::nullopt;
            }
        }
    }

    if (!errors.empty()) {
        for (const auto& err : errors)
            m_logger->error("EntitySpawner::addEntity: {}", err.Message());
        return std::nullopt;
    }

    if (!root.Model()) {
        m_logger->error(
            "EntitySpawner::addEntity: SDF loaded but contains no model.");
        return std::nullopt;
    }

    sdf::Model model = *root.Model();

    // Determine final name: prefer msg.vessel_name, fall back to SDF name.
    std::string desired_name =
        msg.vessel_name.empty() ? model.Name() : msg.vessel_name;
    desired_name = resolveUniqueName(desired_name);
    model.SetName(desired_name);

    // Create entity in the simulation.
    gz::sim::Entity entity = m_creator->CreateEntities(&model);
    if (entity == gz::sim::kNullEntity) {
        m_logger->error(
            "EntitySpawner::addEntity: CreateEntities failed for '{}'",
            desired_name);
        return std::make_tuple(
            static_cast<uint16_t>(gz::sim::kNullEntity),
            std::string{});
    }

    m_creator->SetParent(entity, m_world_entity);
    m_logger->info(
        "EntitySpawner::addEntity: Created entity [{}] named '{}'",
        entity,
        desired_name);

    // Teleport to the requested pose.
    lotusim_msgs::msg::MASCmd move_msg = msg;
    move_msg.entity = entity;
    move_msg.cmd_type = lotusim_msgs::msg::MASCmd::MOVE_CMD;
    moveEntity(move_msg);

    return std::make_tuple(static_cast<uint16_t>(entity), desired_name);
}

bool EntitySpawner::moveEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    try {
        gz::sim::Entity vessel_entity = gz::sim::kNullEntity;

        if (msg.entity) {
            vessel_entity = msg.entity;
        } else if (!msg.vessel_name.empty()) {
            std::shared_lock<std::shared_mutex> lock(m_variable_mutex);
            auto it = m_vessels_entities.find(msg.vessel_name);
            if (it != m_vessels_entities.end())
                vessel_entity = it->second;
        }

        if (vessel_entity == gz::sim::kNullEntity) {
            m_logger->error(
                "EntitySpawner::moveEntity: No entity or vessel_name provided "
                "(vessel='{}', entity={})",
                msg.vessel_name,
                msg.entity);
            return false;
        }

        gz::math::Pose3d pose(
            msg.vessel_position.position.x,
            msg.vessel_position.position.y,
            msg.vessel_position.position.z,
            msg.vessel_position.orientation.w,
            msg.vessel_position.orientation.x,
            msg.vessel_position.orientation.y,
            msg.vessel_position.orientation.z);

        // Override XY (and optionally Z) from geo-point if provided.
        if (msg.geo_point.latitude != 0.0 || msg.geo_point.longitude != 0.0 ||
            msg.geo_point.altitude != 0.0 || msg.heading != 0.0) {
            auto xyz_opt = lotusim::common::LatLongToXY(
                m_ecm,
                msg.geo_point.latitude,
                msg.geo_point.longitude,
                msg.geo_point.altitude);

            if (xyz_opt) {
                pose.Pos().X() = std::get<0>(*xyz_opt);
                pose.Pos().Y() = std::get<1>(*xyz_opt);
                pose.Pos().Z() = std::get<2>(*xyz_opt);
            } else {
                pose.Pos().Z() = msg.geo_point.altitude;
            }
            pose.Rot() = gz::math::Quaterniond(0.0, 0.0, msg.heading);
        }

        if (!m_ecm.SetComponentData<gz::sim::components::Pose>(
                vessel_entity,
                pose)) {
            m_logger->warn(
                "EntitySpawner::moveEntity: SetComponentData failed "
                "(vessel='{}', entity={})",
                msg.vessel_name,
                msg.entity);
        }

        m_ecm.SetChanged(
            vessel_entity,
            gz::sim::components::Pose::typeId,
            gz::sim::ComponentState::OneTimeChange);

    } catch (const std::out_of_range&) {
        m_logger->error(
            "EntitySpawner::moveEntity: Entity not found "
            "(vessel='{}', entity={})",
            msg.vessel_name,
            msg.entity);
        return false;
    } catch (...) {
        m_logger->error(
            "EntitySpawner::moveEntity: Unknown error "
            "(vessel='{}', entity={})",
            msg.vessel_name,
            msg.entity);
        return false;
    }
    return true;
}

bool EntitySpawner::deleteEntity(const lotusim_msgs::msg::MASCmd& msg)
{
    try {
        gz::sim::Entity vessel_entity = gz::sim::kNullEntity;
        {
            std::unique_lock<std::shared_mutex> lock(m_variable_mutex);

            if (msg.entity) {
                vessel_entity = msg.entity;
            } else if (!msg.vessel_name.empty()) {
                auto it = m_vessels_entities.find(msg.vessel_name);
                if (it != m_vessels_entities.end())
                    vessel_entity = it->second;
            }

            if (vessel_entity == gz::sim::kNullEntity) {
                m_logger->error(
                    "EntitySpawner::deleteEntity: No entity or vessel_name "
                    "provided (vessel='{}', entity={})",
                    msg.vessel_name,
                    msg.entity);
                return false;
            }

            // Remove from maps immediately so no one can reference the
            // entity between now and GZ processing the removal request.
            if (auto name_it = m_vessels_names.find(vessel_entity);
                name_it != m_vessels_names.end()) {
                m_vessels_entities.erase(name_it->second);
                m_vessels_names.erase(name_it);
            }
        }

        m_creator->RequestRemoveEntity(vessel_entity);
        return true;

    } catch (...) {
        m_logger->error(
            "EntitySpawner::deleteEntity: Unknown error "
            "(vessel='{}', entity={})",
            msg.vessel_name,
            msg.entity);
        return false;
    }
}

void EntitySpawner::deleteAllEntities()
{
    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);

    for (auto& [entity, name] : m_vessels_names) {
        m_creator->RequestRemoveEntity(entity);
        m_logger->info(
            "EntitySpawner::deleteAllEntities: Removing entity [{}] '{}'",
            entity,
            name);
    }
}

void EntitySpawner::registerNewEntity(
    gz::sim::Entity entity,
    const std::string& name)
{
    {
        std::unique_lock<std::shared_mutex> lock(m_variable_mutex);
        m_vessels_entities[name] = entity;
        m_vessels_names[entity] = name;
    }

    // Enable velocity reporting on the model's base_link.
    auto links =
        m_ecm.ChildrenByComponents(entity, gz::sim::components::Link());

    for (auto link_entity : links) {
        auto name_comp =
            m_ecm.Component<gz::sim::components::Name>(link_entity);
        if (name_comp &&
            name_comp->Data().find("base_link") != std::string::npos) {
            gz::sim::Link link(link_entity);
            link.EnableVelocityChecks(m_ecm);
            break;
        }
    }

    m_logger->info(
        "EntitySpawner::registerNewEntity: Registered entity [{}] '{}'",
        entity,
        name);
}

void EntitySpawner::unregisterEntity(gz::sim::Entity entity)
{
    std::unique_lock<std::shared_mutex> lock(m_variable_mutex);

    auto it = m_vessels_names.find(entity);
    if (it == m_vessels_names.end())
        return;

    m_logger->info(
        "EntitySpawner::unregisterEntity: Removed entity [{}] '{}'",
        entity,
        it->second);

    m_vessels_entities.erase(it->second);
    m_vessels_names.erase(it);
}

std::string EntitySpawner::resolveUniqueName(const std::string& desired) const
{
    if (desired.empty())
        return desired;

    // Check if name already exists as a direct child of the world.
    auto taken = [&](const std::string& name) {
        return gz::sim::kNullEntity !=
               m_ecm.EntityByComponents(
                   gz::sim::components::Name(name),
                   gz::sim::components::ParentEntity(m_world_entity));
    };

    if (!taken(desired))
        return desired;

    std::string candidate;
    for (int i = 0;; ++i) {
        candidate = desired + "_" + std::to_string(i);
        if (!taken(candidate))
            return candidate;
    }
}

}  // namespace lotusim::mas