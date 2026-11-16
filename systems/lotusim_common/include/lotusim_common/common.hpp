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
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
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
// The angle, frame and geographic conversions used to be declared here;
// kept so that code including common.hpp still compiles.
#include "lotusim_common/transforms.hpp"

namespace lotusim::common {

enum class RandomisedType
{
    NONE,
    RANDOM
};

class PowerStateRegistry {
public:
    /**
     * @brief Returns the single shared instance of the registry
     *        Defined in common.cpp so all shared libraries (.so) that link
     *        against lotusim_common share the same instance
     */
    static PowerStateRegistry& instance();

    /**
     * @brief Sets the power state for a sensor
     *        Called by SensorPowerConsumer::update() every sim tick from
     *        PowerManagerInstance::Update(), which runs before PostUpdate().
     * @param key     vessel_name + "/" + sensor_name
     * @param powered true = sensor is active, false = sensor is deactivated
     */
    void set(const std::string& key, bool powered);

    /**
     * @brief Returns the power state for a sensor
     *        Called by LotusimSensorPlugin::PostUpdate() before calling
     *        UpdateSensor(). if false the sensor update is skipped entirely.
     * @param key        vessel_name + "/" + sensor_name
     * @param defaultVal returned when the key is not in the registry
     *                   defaults to true so sensors without a power manager
     *                   always run normally.
     */
    bool get(const std::string& key, bool defaultVal = true);

private:
    PowerStateRegistry() = default;
    std::mutex m_mutex;
    std::unordered_map<std::string, bool> m_flags;
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
 * @brief Check that a name is a valid ROS name token: non-empty, not starting
 * with a digit, and made only of letters, digits and underscores. Use it
 * before building a topic, action or entity name from user input, as an
 * invalid one makes rclcpp throw.
 *
 * @param name
 * @return true if the name is valid
 */
bool isValidRosName(const std::string& name);

}  // namespace lotusim::common
#endif  // COMMON_HPP