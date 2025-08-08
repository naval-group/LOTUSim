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

std::optional<std::pair<gz::sim::Entity, std::string>> getModelName(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity &_entity)
{
    gz::sim::Entity entity = _entity;

    while (true) {
        if (_ecm.EntityHasComponentType(
                entity,
                gz::sim::components::Model::typeId)) {
            auto nameComp = _ecm.Component<gz::sim::components::Name>(entity);
            if (nameComp) {
                return std::make_pair(entity, nameComp->Data());
            } else {
                return std::nullopt;
            }
        }

        auto parentComp =
            _ecm.Component<gz::sim::components::ParentEntity>(entity);
        if (!parentComp) {
            return std::nullopt;
        }

        entity = parentComp->Data();
    }
    return std::nullopt;
}
}  // namespace lotusim::common