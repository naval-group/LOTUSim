/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_common/entity_group.hpp"

namespace lotusim::common {

uint64_t EntityGraph::find(const uint64_t& entity)
{
    if (m_parent.find(entity) == m_parent.end()) {
        m_parent[entity] = entity;
    }

    if (m_parent[entity] != entity) {
        m_parent[entity] = find(m_parent[entity]);
    }
    return m_parent[entity];
}

void EntityGraph::addPair(const uint64_t& entity1, const uint64_t& entity2)
{
    uint64_t root1 = find(entity1);
    uint64_t root2 = find(entity2);

    if (root1 != root2) {
        m_parent[root1] = root2;
    }
}

bool EntityGraph::areLinked(const uint64_t& entity1, const uint64_t& entity2)
{
    return find(entity1) == find(entity2);
}

std::vector<uint64_t> EntityGraph::getGroup(const uint64_t& entity)
{
    uint64_t root = find(entity);
    std::vector<uint64_t> group;

    for (const auto& [key, _] : m_parent) {
        if (find(key) == root) {
            group.push_back(key);
        }
    }
    return group;
}

std::vector<std::vector<uint64_t>> EntityGraph::getAllSets()
{
    // Group entities by their root
    std::unordered_map<uint64_t, std::vector<uint64_t>> groups;

    for (const auto& [entity, _] : m_parent) {
        uint64_t root = find(entity);
        groups[root].push_back(entity);
    }

    // Convert to vector of vectors
    std::vector<std::vector<uint64_t>> result;
    for (const auto& [root, entities] : groups) {
        result.push_back(entities);
    }

    return result;
}

int EntityGraph::getSetCount()
{
    std::unordered_set<uint64_t> roots;
    for (const auto& [entity, _] : m_parent) {
        roots.insert(find(entity));
    }
    return roots.size();
}

void EntityGraph::clearGraph()
{
    m_parent.clear();
}

}  // namespace lotusim::common