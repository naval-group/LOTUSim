/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COMMON_GEO_TYPES_HPP
#define LOTUSIM_COMMON_GEO_TYPES_HPP

#include <gz/math/Quaternion.hh>

/**
 * @brief Geographic position and pose types.
 *
 * Local Cartesian positions and attitudes stay gz::math::Vector3d and
 * gz::math::Quaterniond. Geographic positions get their own type so that a
 * latitude can never be read as an x: a gz::math::Vector3d holding
 * (lat, lon, alt) looks like a local position to the compiler.
 *
 * Header-only and free of gz-sim, so message formatting code can use it
 * without linking the simulator. The conversions live in transforms.hpp.
 */
namespace lotusim::common {

/// A WGS84 position.
struct GeoPoint {
    /// Latitude, degrees, positive North.
    double latitude = 0.0;
    /// Longitude, degrees, positive East.
    double longitude = 0.0;
    /// Metres from sea level, positive up, negative when submerged.
    double altitude = 0.0;
};

/// A WGS84 position with an attitude, as geographic_msgs/GeoPose.
struct GeoPose {
    GeoPoint position;
    /// Attitude in the ENU world frame (x East, y North, z Up), body FLU.
    gz::math::Quaterniond orientation = gz::math::Quaterniond::Identity;
};

}  // namespace lotusim::common

#endif
