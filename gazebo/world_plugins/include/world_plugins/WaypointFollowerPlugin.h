#ifndef WAYPOINT_FOLLOWER_PLUGIN_HH
#define WAYPOINT_FOLLOWER_PLUGIN_HH

#include <functional>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
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
#include <mutex>
#include <sdf/sdf.hh>
#include <string>
#include <unordered_map>
#include <vector>

namespace liquidai {
namespace gazebo {

/// \brief A plugin that scripts the movement of a model based on waypoints.
/// Note that at this point, the trajectory is always in 2D (x, y).
/// Acceleration will be added to the object until the model is close enough to
/// the waypoint.
///
/// Waypoints may be inserted manually via the <waypoints> element, or
/// generated relative to the model's initial position via the <line> or
/// <circle> elements.  Only one of these options should be used.
///
/// ## System Parameters
///
/// This plugin requires the following SDF parameters:
///
/// Required parameters:
///
/// - `<link_name>`: The name of the link within the model where the
///   force/torque will be applied when moving the vehicle. Heading has to be
///   the heading of the ship
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
/// - `<bearing_tolerance>`: If the bearing to the current waypoint is within
///   +- this tolerance, a torque won't be applied (degree).
///   The default value is 2 deg.
///
/// - `<zero_vel_on_bearing_reached>`: Force angular velocity to be zero when
///   target bearing is reached.
///   Default is false.
///
/// - `<line>`: Element that indicates the model should travel in "line" mode.
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
/// name="liquidai::gazebo::WaypointFollowerPlugin">
/// </ plugin>
///
/// <follower>
///   <loop>true</loop>
///   <waypoints>
///     <waypoint>25 0</waypoint>
///     <waypoint>15 0</waypoint>
///   </waypoints>
/// </follower>
/// <follower>
///   <loop>true</loop>
///   <line>
///     <direction>0</direction>
///     <length>5</length>
///   </line>
/// </follower>
/// <follower>
///   <loop>true</loop>
///   <circle>
///     <radius>2</radius>
///   </circle>
/// </follower>
/// ```

class WaypointFollowerPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemUpdate {
public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:
    // TODO changing the returns to handle error
    bool Load(
        const gz::sim::Entity &_entity,
        const gz::sim::components::ModelSdf *_model,
        gz::sim::EntityComponentManager *_ecm);

private:
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
     * @brief Velocities limit
     *
     */
    std::unordered_map<gz::sim::Entity, double> m_angular_velocities_limits;

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

    /**
     * @brief Vectors of waypoint for the vessel
     *
     */
    std::unordered_map<gz::sim::Entity, std::vector<gz::math::Vector2d>>
        m_waypoints;

    /**
     * @brief The current waypoint the vessel is going
     *
     */
    std::unordered_map<gz::sim::Entity, uint> m_waypoint_state;
};

}  // namespace gazebo
}  // namespace liquidai
#endif