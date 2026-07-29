/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_ENTITY_SPAWNER_HH_
#define LOTUSIM_ENTITY_SPAWNER_HH_

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "lotusim_common/logger.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"

namespace lotusim::mas {

/**
 * @brief Manages the lifecycle of simulation entities (vessels).
 *
 * Owns the vessel name↔entity maps and exposes add / move / delete
 * operations used by both MultiAgentSystem (for ad-hoc MASCmd requests)
 * and ScenarioManager (for bulk scenario spawn / despawn).
 *
 * Thread-safety:
 *   - addEntity / moveEntity / deleteEntity are called from the GZ
 *     PreUpdate thread only, so no locking is needed for ECM writes.
 *   - m_vessels_entities / m_vessels_names are protected by
 *     m_variable_mutex and may be read from any thread (e.g. PostUpdate
 *     pose publisher).
 */
class EntitySpawner {
public:
    /**
     * @param ecm          Reference to the ECM (must outlive this object).
     * @param world_entity GZ entity of the world.
     * @param creator      SdfEntityCreator (must outlive this object).
     * @param logger       Shared spdlog logger.
     */
    EntitySpawner(
        gz::sim::EntityComponentManager& ecm,
        gz::sim::Entity world_entity,
        std::shared_ptr<gz::sim::SdfEntityCreator> creator,
        std::shared_ptr<spdlog::logger> logger);

    ~EntitySpawner() = default;

    EntitySpawner(const EntitySpawner&) = delete;
    EntitySpawner& operator=(const EntitySpawner&) = delete;
    EntitySpawner(EntitySpawner&&) = delete;
    EntitySpawner& operator=(EntitySpawner&&) = delete;

    /**
     * @brief Spawn a new entity from a MASCmd.
     *
     * Expects either:
     *  - msg.sdf_string  — a complete SDF string (msg.model_name empty), or
     *  - msg.model_name  — asset name under $LOTUSIM_MODELS_PATH, with an
     *                      optional <lotus_param> block in msg.sdf_string.
     *
     * @return {entity_id, final_name} on success, std::nullopt on failure.
     */
    std::optional<std::tuple<uint16_t, std::string>> addEntity(
        const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Teleport an existing entity to the pose / geo-point in msg.
     *
     * Identifies the target by msg.entity (preferred) or msg.vessel_name.
     *
     * @return true on success.
     */
    bool moveEntity(const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Request removal of an entity.
     *
     * Identifies the target by msg.entity (preferred) or msg.vessel_name.
     * The entity is removed from the internal maps immediately; GZ removes
     * it from the simulation on the next update step.
     *
     * @return true on success.
     */
    bool deleteEntity(const lotusim_msgs::msg::MASCmd& msg);

    /**
     * @brief Request removal of all currently tracked entities.
     *
     * Clears the internal maps immediately; GZ removes each entity
     * from the simulation on the next update step.
     */
    void deleteAllEntities();

    /**
     * @brief Register a newly spawned entity once GZ has confirmed it exists.
     *
     * Called from MultiAgentSystem::Update via EachNew<ModelSdf>.
     * Also enables velocity checks on the model's base_link.
     */
    void registerNewEntity(gz::sim::Entity entity, const std::string& name);

    /**
     * @brief Unregister an entity that GZ has removed.
     *
     * Called from MultiAgentSystem::Update via EachRemoved<ModelSdf>.
     */
    void unregisterEntity(gz::sim::Entity entity);

    /**
     * @brief Snapshot of entity→name map for pose publishing.
     *
     * Caller must hold the shared lock returned by sharedLock() for the
     * duration of the iteration.
     */
    const std::unordered_map<gz::sim::Entity, std::string>& vesselNames() const
    {
        return m_vessels_names;
    }

    /**
     * @brief Acquire a shared (read) lock on the vessel maps.
     */
    [[nodiscard]] std::shared_lock<std::shared_mutex> sharedLock() const
    {
        return std::shared_lock<std::shared_mutex>(m_variable_mutex);
    }

    /**
     * @brief Acquire a unique (read) lock on the vessel maps.
     */
    [[nodiscard]] std::unique_lock<std::shared_mutex> uniqueLock() const
    {
        return std::unique_lock<std::shared_mutex>(m_variable_mutex);
    }

private:
    /**
     * @brief Resolve unique name — appends _0, _1, … if already taken.
     */
    std::string resolveUniqueName(const std::string& desired) const;

private:
    std::shared_ptr<spdlog::logger> m_logger;

    gz::sim::EntityComponentManager& m_ecm;

    gz::sim::Entity m_world_entity;

    std::shared_ptr<gz::sim::SdfEntityCreator> m_creator;

    mutable std::shared_mutex m_variable_mutex;

    /// vessel name  →  GZ entity
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_entities;

    /// GZ entity  →  vessel name
    std::unordered_map<gz::sim::Entity, std::string> m_vessels_names;
};

}  // namespace lotusim::mas
#endif  // LOTUSIM_ENTITY_SPAWNER_HH_