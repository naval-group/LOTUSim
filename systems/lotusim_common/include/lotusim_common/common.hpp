/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COMMON_HPP_
#define LOTUSIM_COMMON_HPP_

#include <algorithm>
#include <cmath>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "gz/math/SphericalCoordinates.hh"
#include "std_msgs/msg/header.hpp"

namespace lotusim::common {

enum class RandomisedType
{
    NONE,
    RANDOM
};

/**
 * @brief Random shuffle for vector
 *
 * @tparam T
 * @param vector
 * @param _type
 */
template <typename T>
void shuffleOrder(
    std::vector<T>& vector,
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

/**
 * @brief Test for equal pose
 *
 * @param _a
 * @param _b
 * @return true
 * @return false
 */
bool pose3Eql(const gz::math::Pose3d& _a, const gz::math::Pose3d& _b);

/**
 * @brief Get the World Name object
 *
 * @param _ecm
 * @return std::string
 */
std::string getWorldName(const gz::sim::EntityComponentManager& _ecm);

/**
 * @brief Get the Model Name object regardless of any child enetity
 *
 * @param _ecm
 * @param _entity
 * @return std::optional<std::pair<gz::sim::Entity, std::string>>
 */
std::optional<std::pair<gz::sim::Entity, std::string>> getModelName(
    const gz::sim::EntityComponentManager& _ecm,
    const gz::sim::Entity& _entity);

/**
 * @brief Generate ROS2 Header message
 *
 * @param _time
 * @return std_msgs::msg::Header
 */
std_msgs::msg::Header generateHeaderMessage(
    const std::chrono::steady_clock::duration& _time);

/**
 * @brief Convert latitude/longitude/altitude to local XYZ (ENU metres).
 *
 * @param _ecm
 * @param lat degrees
 * @param lon degrees
 * @param alt altitude in metres (default 0)
 * @return std::optional<std::tuple<double, double, double>> (x, y, z)
 */
std::optional<std::tuple<double, double, double>> LatLongToXY(
    const gz::sim::EntityComponentManager& _ecm,
    double lat,
    double lon,
    double alt = 0.0);

/**
 * @brief Convert local XYZ (ENU metres) to latitude/longitude/altitude.
 *
 * @param _ecm Entity Component Manager used to retrieve spherical coordinates
 * @param x Local X coordinate in metres
 * @param y Local Y coordinate in metres
 * @param z Local Z coordinate in metres (default 0)
 * @return std::optional<std::tuple<double, double, double>> (lat deg, lon deg,
 * alt metres), or std::nullopt if spherical coordinates are unavailable
 */
std::optional<std::tuple<double, double, double>> XYToLatLong(
    const gz::sim::EntityComponentManager& _ecm,
    double x,
    double y,
    double z = 0.0);

/**
 * @brief Get the SDF Element with case insensitive name
 *
 * @param parent
 * @param name
 * @return sdf::ElementPtr
 */
sdf::ElementPtr getElementCaseInsensitive(
    sdf::ElementPtr parent,
    const std::string& name);

/**
 * @brief Convert string to upper case
 *
 * @param str
 * @return std::string
 */
std::string toUpper(std::string str);

/**
 * @brief Convert radians to degrees
 *
 * @param rad angle in radians
 * @return double angle in degrees
 */
inline double radToDeg(double rad)
{
    return rad * (180.0 / M_PI);
}

/**
 * @brief Convert degrees to radians
 *
 * @param deg angle in degrees
 * @return double angle in radians
 */
inline double degToRad(double deg)
{
    return deg * (M_PI / 180.0);
}

}  // namespace lotusim::common
#endif  // COMMON_HPP