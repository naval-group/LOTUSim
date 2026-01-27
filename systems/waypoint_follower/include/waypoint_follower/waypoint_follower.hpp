/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef WAYPOINT_FOLLOWER_PLUGIN_HPP_
#define WAYPOINT_FOLLOWER_PLUGIN_HPP_

#include <functional>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_msgs/msg/waypoint_follower_status.hpp"
#include "lotusim_msgs/srv/set_waypoints.hpp"
#include "std_srvs/srv/empty.hpp"

namespace lotusim::gazebo {
/// \brief A plugin that scripts the movement of a model based on waypoints.
/// Note that at this point, the trajectory is always in 2D (x, y).
/// Acceleration will be added to the object until the model is close enough
/// to the waypoint.
///
/// Waypoints may be inserted manually via the <waypoints> element, or
/// generated relative to the model's initial position via the <line> or
/// <circle> elements.  Only one of these options should be used.
///
/// All waypoint follower will have a  `<model_name>/waypoints` topic for
/// use to send in geographic_msgs/GeoPath messages to override the given
/// waypoints
///
/// ## System Parameters
///
/// This plugin requires the following SDF parameters:
///
/// Required parameters:
///
/// - `<follower>`: Bool to enable waypoint following
///
/// Optional parameters:
///
/// - `<loop>`: When true, all waypoints will be visited continously in a
///   circular pattern. If false, the model will stop when the
///   last waypoint is reached. Note, that if the vehicle moves,
///   it will still try to reach the very last waypoint.
///
/// - `<waypoints>`: Element specifying the set of waypoints that the
///   the model should navigate through. This block should contain
///   at least one of these blocks:
///   - `<waypoint>`: This block should contain the X, Y of a waypoint.
///
/// - `<range_tolerance>`: The current waypoint is considered reached if the
///   distance to it is within +- this tolerance (m).
///   The default value is 0.5m.
///
/// - `<bearing_tolerance>`: If the bearing to the current waypoint is
/// within
///   +- this tolerance, a torque won't be applied (degree).
///   The default value is 2 deg.
///
/// - `<zero_vel_on_bearing_reached>`: Force angular velocity to be zero
/// when
///   target bearing is reached.
///   Default is false.
///
/// - `<line>`: Element that indicates the model should travel in "line"
/// mode.
///   The block should contain the relative direction and distance from
///   the initial position in which the vehicle should move, specified
///   in the world frame.
///   - `<direction>`: Relative direction (radians) in the world frame for
///     the vehicle to travel.
///   - `<length>`: Distance (meters) for the vehicle to travel.
///
/// - `<circle>`: Element that indicates the model should travel in "circle"
///   mode. The block should contain the desired radius of the circle about
///   the vehicle's initial position
///   - `<radius>`: Radius (meters) of circular path to travel.
///
/// Here are three examples:
/// ```
/// <plugin filename="waypoint_plugin"
/// name="lotusim::gazebo::WaypointFollowerPlugin">
/// </ plugin>
///
///< waypoint_follower>
///  <follower>
///    <loop>true</loop>
///    <linear_acceleration_limit>0.1</linear_acceleration_limit>
///    <angular_acceleration_limit>0.005</angular_acceleration_limit>
///    <angular_velocity_limit>0.01</angular_velocity_limit>
///    <waypoints>
///      <waypoint>25 0</waypoint>
///      <waypoint>15 0</waypoint>
///    </waypoints>
///  </follower>
///  <follower>
///    <loop>true</loop>
///    <line>
///      <direction>0</direction>
///      <length>5</length>
///    </line>
///  </follower>
///  <follower>
///    <loop>true</loop>
///    <circle>
///      <radius>2</radius>
///    </circle>
///  </follower>
///</waypoint_follower>
/// ```

class WaypointFollowerPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemUpdate {
public:
    WaypointFollowerPlugin();

    ~WaypointFollowerPlugin();

    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    // TODO changing the returns to handle error
    bool load(
        const gz::sim::Entity& _entity,
        sdf::ElementPtr _lotus_param,
        gz::sim::EntityComponentManager& _ecm);

    // Sets up ROS subscriptions and publishers for a given model/entity
    void setupRosForModel(
        const gz::sim::Entity& entity,
        const std::string& model_name);

    // Stop the vessel and stop updating
    bool stopVessel(const gz::sim::Entity& entity);

    // Publish the vessel status
    void publishStatus(const gz::sim::Entity& entity);

private:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    std::unordered_map<gz::sim::Entity, std::string> m_vessel_name;

    /**
     * @brief Mapping of entity to velocities
     *
     */
    std::unordered_map<gz::sim::Entity, gz::math::Vector2d> m_velocities;

    /**
     * @brief Linear accel limit for each vessel
     *
     */
    std::unordered_map<gz::sim::Entity, double> m_linear_accel_limit;

    /**
     * @brief Angular accel limit for each vessel
     *
     */
    std::unordered_map<gz::sim::Entity, double> m_angular_accel_limit;

    /**
     * @brief Velocities limit, Min and Max
     *
     */
    std::unordered_map<gz::sim::Entity, gz::math::Vector2d>
        m_linear_velocities_limits;

    /**
     * @brief Linear velocity pid settings
     */
    std::unordered_map<gz::sim::Entity, std::array<double, 3>> m_linear_pid;

    /**
     * @brief Velocities limit
     *
     */
    std::unordered_map<gz::sim::Entity, double> m_angular_velocities_limits;

    /**
     * @brief Angular pid settings
     */
    std::unordered_map<gz::sim::Entity, std::array<double, 3>> m_angular_pid;

    /**
     * @brief Flag for vessel looping
     *
     */
    std::unordered_map<gz::sim::Entity, bool> m_loop;

    /**
     * @brief Range tolerance to the end point
     *
     */
    std::unordered_map<gz::sim::Entity, double> m_rangeTolerance;

    std::unordered_map<gz::sim::Entity, sdf::ElementPtr> m_model_load_queue;

    /**
     * @brief Mutex for all waypoint params
     * Mutex for m_waypoints, m_waypoints_geo, m_waypoint_state
     *
     */
    std::mutex m_waypoint_mutex;

    /**
     * @brief Vectors of waypoint for the vessel in local XY coord system
     *
     */
    std::unordered_map<gz::sim::Entity, std::vector<gz::math::Vector2d>>
        m_waypoints;

    /**
     * @brief Vectors of geographic waypoint (lat, lon) for the vessel
     *
     */
    std::unordered_map<
        gz::sim::Entity,
        std::vector<geographic_msgs::msg::GeoPoint>>
        m_waypoints_geo;

    /**
     * @brief The current waypoint the vessel is going
     *
     */
    std::unordered_map<gz::sim::Entity, uint> m_waypoint_state;

    /**
     * @brief Entities that should stop being processed by the waypoint
     * follower..
     *
     */
    std::set<gz::sim::Entity> m_entities_to_remove;

    std::map<gz::sim::Entity, double> m_distance_error_integral;
    std::map<gz::sim::Entity, double> m_distance_error_previous;

    // Implementing pid for heading control
    std::unordered_map<gz::sim::Entity, double> m_heading_integral;
    std::unordered_map<gz::sim::Entity, double> m_prev_heading_error;
    std::unordered_map<gz::sim::Entity, double> m_prev_yaw;
    std::unordered_map<gz::sim::Entity, gz::math::Vector2d> m_prevGoal;

    /**
     * @brief GZ world entity
     *
     */
    gz::sim::Entity m_world_entity;

    /**
     * @brief GZ world name
     *
     */
    std::string m_world_name;

    /**
     * @brief ROS node
     *
     */
    rclcpp::Node::SharedPtr m_ros_node;

    std::shared_ptr<std::thread> m_ros_node_thread;

    std::vector<rclcpp::Service<lotusim_msgs::srv::SetWaypoints>::SharedPtr>
        m_waypoint_services;

    std::vector<rclcpp::Service<std_srvs::srv::Empty>::SharedPtr>
        m_waypoint_stop_services;

    gz::math::SphericalCoordinates m_origin_spherical;

    std::unordered_map<
        gz::sim::Entity,
        rclcpp::Publisher<lotusim_msgs::msg::WaypointFollowerStatus>::SharedPtr>
        m_waypoint_pub;

    rclcpp::TimerBase::SharedPtr m_status_timer;
    uint16_t m_status_update_period;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
};

}  // namespace lotusim::gazebo
#endif