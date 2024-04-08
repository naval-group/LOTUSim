#ifndef __COORDINATE_SYSTEM_UTIL_HH__
#define __COORDINATE_SYSTEM_UTIL_HH__

#include "gz/sim/components/SphericalCoordinates.hh"
#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/System.hh>

bool TransformCartesianToSpherical(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity _entity,
    const gz::math::Vector3d &cart_input,
    gz::math::Vector3d &sc_output)
{
    try {
        auto scComp =
            _ecm.Component<gz::sim::v7::components::SphericalCoordinates>(
                _entity);
        sc_output = scComp->Data().PositionTransform(
            cart_input,
            gz::math::SphericalCoordinates::LOCAL2,
            gz::math::SphericalCoordinates::SPHERICAL);
    }
    catch (...) {
        return false;
    }
    return true;
}

bool TransformSphericalToCartesian(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity _entity,
    const gz::math::Vector3d &sc_input,
    gz::math::Vector3d &cart_output)
{
    try {
        auto scComp =
            _ecm.Component<gz::sim::v7::components::SphericalCoordinates>(
                _entity);
        cart_output = scComp->Data().PositionTransform(
            sc_input,
            gz::math::SphericalCoordinates::SPHERICAL,
            gz::math::SphericalCoordinates::LOCAL2);
    }
    catch (...) {
        return false;
    }
    return true;
}

#endif