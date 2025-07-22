#include "lotusim_common/common.hpp"

namespace lotusim::common {

bool pose3Eql(const gz::math::Pose3d &_a, const gz::math::Pose3d &_b)
{
    return _a.Pos().Equal(_b.Pos(), 1e-6) &&
           gz::math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
           gz::math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
           gz::math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
           gz::math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
}

std::string getWorldName(const gz::sim::EntityComponentManager &_ecm)
{
    std::string world_name = "";
    _ecm.Each<gz::sim::components::Name, gz::sim::components::World>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::World *) -> bool {
            world_name = _name->Data();
            return true;
        });
    return world_name;
}

}  // namespace lotusim::common