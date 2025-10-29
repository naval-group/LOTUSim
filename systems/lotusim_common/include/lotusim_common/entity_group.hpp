#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lotusim::common {

/**
 * @brief EntityGraph class to manage groups of linked entities.
 *
 */
class EntityGraph {
public:
    void addPair(const uint64_t& entity1, const uint64_t& entity2);

    bool areLinked(const uint64_t& entity1, const uint64_t& entity2);

    std::vector<uint64_t> getGroup(const uint64_t& entity);

    std::vector<std::vector<uint64_t>> getAllSets();

    int getSetCount();

    void clearGraph();

private:
    uint64_t find(const uint64_t& entity);

    std::unordered_map<uint64_t, uint64_t> m_parent;
};

}  // namespace lotusim::common