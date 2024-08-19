#ifndef __COMMON_HH__
#define __COMMON_HH__

#include <algorithm>
#include <random>
#include <vector>

namespace liquidai {
namespace gazebo {

enum class RandomisedType
{
    NONE,
    RANDOM
};

void shuffleOrder(std::vector<uint64_t> &_entities, RandomisedType _type)
{
    switch (_type) {
        case (RandomisedType::RANDOM): {
            std::random_device rd;
            std::default_random_engine rng(rd());
            std::shuffle(_entities.begin(), _entities.end(), rng);
        }
        default: {
        }
    }
}
}  // namespace gazebo
}  // namespace liquidai
#endif