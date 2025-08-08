#ifndef COMMON_HPP
#define COMMON_HPP

#include <algorithm>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace lotusim::common {

enum class RandomisedType
{
    NONE,
    RANDOM
};

template <typename T>
void shuffleOrder(
    std::vector<T> &vector,
    RandomisedType _type = RandomisedType::RANDOM)
{
    switch (_type) {
        case RandomisedType::RANDOM: {
            std::random_device rd;
            std::default_random_engine rng(rd());
            std::shuffle(vector.begin(), vector.end(), rng);
            break;
        }
        default: {
            // Handle other types or leave empty
            break;
        }
    }
}
bool pose3Eql(const gz::math::Pose3d &_a, const gz::math::Pose3d &_b);

std::string getWorldName(const gz::sim::EntityComponentManager &_ecm);

std::optional<std::pair<gz::sim::Entity, std::string>> getModelName(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity &_entity);
}  // namespace lotusim::common

#endif  // COMMON_HPP