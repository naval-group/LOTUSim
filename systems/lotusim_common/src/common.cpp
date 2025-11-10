/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_common/common.hpp"
namespace lotusim::common {

bool pose3Eql(const gz::math::Pose3d& _a, const gz::math::Pose3d& _b)
{
    return _a.Pos().Equal(_b.Pos(), 1e-6) &&
           gz::math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
           gz::math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
           gz::math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
           gz::math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
}

std::string getWorldName(const gz::sim::EntityComponentManager& _ecm)
{
    std::string world_name = "";
    _ecm.Each<gz::sim::components::Name, gz::sim::components::World>(
        [&](const gz::sim::Entity&,
            const gz::sim::components::Name* _name,
            const gz::sim::components::World*) -> bool {
            world_name = _name->Data();
            return true;
        });
    return world_name;
}

std::optional<std::pair<gz::sim::Entity, std::string>> getModelName(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::sim::Entity& _entity)
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

std_msgs::msg::Header generateHeaderMessage(
    const std::chrono::steady_clock::duration& _time)
{
    std_msgs::msg::Header msg;
    auto simTimeNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(_time).count();
    msg.stamp.sec = static_cast<int32_t>(simTimeNs / 1000000000);
    msg.stamp.nanosec = static_cast<uint32_t>(simTimeNs % 1000000000);
    msg.frame_id = "world";
    return msg;
}

std::optional<std::tuple<double, double>> XYFromLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    double lat,
    double lon)
{
    gz::math::Angle lat0, lon0;
    gz::sim::Entity worldEntity;
    gz::math::SphericalCoordinates sphCoords;
    _ecm.Each<gz::sim::components::Name, gz::sim::components::World>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Name*,
            const gz::sim::components::World*) -> bool {
            worldEntity = _entity;
            return true;
        });
    if (auto sphComp =
            _ecm.Component<gz::sim::components::SphericalCoordinates>(
                worldEntity)) {
        lat0 = sphComp->Data().LatitudeReference();
        lon0 = sphComp->Data().LongitudeReference();
    }
    sphCoords.SetLatitudeReference(lat0);
    sphCoords.SetLongitudeReference(lon0);
    sphCoords.SetElevationReference(0);

    gz::math::Vector3d xyz =
        sphCoords.LocalFromSphericalPosition(gz::math::Vector3d{lat, lon, 0});

    return std::make_tuple(xyz.X(), xyz.Y());
}

sdf::ElementPtr getElementCaseInsensitive(
    sdf::ElementPtr parent,
    const std::string& name)
{
    std::string capitalized = toUpper(name);
    auto element = parent->GetFirstElement();
    while (element) {
        auto element_name = toUpper(element->GetName());
        if (element_name == capitalized) {
            return element;
        }
        element = element->GetNextElement();
    }
    return nullptr;
}

std::string toUpper(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::toupper(c);
    });
    return str;
}

}  // namespace lotusim::common