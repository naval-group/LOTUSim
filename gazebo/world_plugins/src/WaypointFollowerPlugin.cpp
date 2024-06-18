
#include "world_plugins/WaypointFollowerPlugin.h"

namespace liquidai {
namespace gazebo {
using namespace std::placeholders;

//////////////////////////////////////////////////
bool WaypointFollowerPlugin::Load(
    const gz::sim::Entity &_entity,
    const gz::sim::components::ModelSdf *_model,
    gz::sim::EntityComponentManager *_ecm)
{
    sdf::ElementPtr _sdf = _model->Data().Element();

    if (!_sdf->HasElement("follower")) {
        return true;
    }
    else {
        _sdf = _sdf->GetElement("follower");
    }

    gz::math::Pose3d pose =
        _ecm->Component<gz::sim::components::Pose>(_entity)->Data();

    // Parse the optional <waypoints> element.
    std::vector<gz::math::Vector2d> waypoint;
    if (_sdf->HasElement("waypoints")) {
        auto waypointsElem = _sdf->GetElement("waypoints");

        // We need at least one waypoint
        if (!waypointsElem->HasElement("waypoint")) {
            gzerr << "TrajectoryFollower: Unable to find "
                     "<waypoints><waypoint> "
                  << "element in SDF." << std::endl;
            return true;
        }
        auto waypointElem = waypointsElem->GetElement("waypoint");
        while (waypointElem) {
            gz::math::Vector2d position =
                waypointElem->Get<gz::math::Vector2d>();

            // Save the position.
            waypoint.push_back(position);

            // Print some debugging messages
            gzdbg << "Waypoint, Local: X = " << position.X()
                  << " Y = " << position.Y() << std::endl;

            waypointElem = waypointElem->GetNextElement("waypoint");
        }
    }
    // If no waypoints present, check for the <circle> element and parse.
    else if (_sdf->HasElement("circle")) {
        gzdbg << "Circle element activated" << std::endl;
        auto circleElem = _sdf->GetElement("circle");

        if (!circleElem->HasElement("radius")) {
            gzerr << "No <circle><radius> specified" << std::endl;
            return true;
        }

        // Parse the required <radius> field.
        double radius = circleElem->Get<double>("radius");

        // Get the current model position in global coordinates. Create
        // local vectors that represent a path along a rough circle.
        gz::math::Vector2d position(pose.X(), pose.Y());
        double angle = 0;
        gz::math::Vector2d vec(radius, 0);
        for (unsigned int i = 0u; i < 8; ++i) {
            // Add the local vector to the current position.
            // Store global position as a waypoint.
            waypoint.push_back(position + vec);
            angle += 2 * GZ_PI / 8;
            vec.Set(radius * cos(angle), radius * sin(angle));
            gzdbg << "Entered circle waypoint " << position + vec << std::endl;
        }
    }
    // If no waypoints or circle, check for the <line> element and parse.
    else if (_sdf->HasElement("line")) {
        auto lineElem = _sdf->GetElement("line");
        // Parse the required <direction> field.
        if (!lineElem->HasElement("direction")) {
            gzerr << "No <line><direction> specified" << std::endl;
            return true;
        }
        gz::math::Angle direction = lineElem->Get<gz::math::Angle>("direction");

        // Parse the required <length> field.
        if (!lineElem->HasElement("length")) {
            gzerr << "No <line><length> specified" << std::endl;
            return true;
        }
        auto length = lineElem->Get<double>("length");

        // Create a relative vector in the direction of "direction" and of
        // length "length".
        gz::math::Vector3d lineVec(
            length * cos(direction.Radian()),
            length * sin(direction.Radian()),
            0);
        gz::math::Vector2d position(pose.X(), pose.Y());
        // Add the initial model position and calculated endpoint as
        // waypoints.
        waypoint.push_back(position);
        gz::math::Vector3d p = pose.CoordPositionAdd(lineVec);
        gz::math::Vector2d p2D = {p.X(), p.Y()};
        waypoint.push_back(p2D);
        gzdbg << "Entered line waypoints " << position << ", " << p2D
              << std::endl;
    }

    m_waypoints[_entity] = waypoint;

    // Parse the optional <loop> element.
    if (_sdf->HasElement("loop"))
        m_loop[_entity] = _sdf->Get<bool>("loop");

    // Parse the optional <range_tolerance> element.
    if (_sdf->HasElement("range_tolerance")) {
        double rangeTolerance = _sdf->Get<double>("range_tolerance");
        m_rangeTolerance[_entity] = rangeTolerance;
    }
    else {
        m_rangeTolerance[_entity] = 0.5;
    }

    if (_sdf->HasElement("linear_accel_limit")) {
        double linear_accel_limit = _sdf->Get<double>("linear_accel_limit");
        m_linear_accel_limit[_entity] = linear_accel_limit;
    }
    else {
        m_linear_accel_limit[_entity] = 0.5;
    }

    if (_sdf->HasElement("angular_accel_limit")) {
        double angular_accel_limit = _sdf->Get<double>("angular_accel_limit");
        m_angular_accel_limit[_entity] = angular_accel_limit;
    }
    else {
        m_angular_accel_limit[_entity] = 0.5;
    }

    if (_sdf->HasElement("linear_velocities_limits")) {
        m_linear_velocities_limits[_entity] =
            _sdf->Get<gz::math::Vector2d>("linear_velocities_limits");
    }
    else {
        m_linear_velocities_limits[_entity] = gz::math::Vector2d(1, 10);
    }

    if (_sdf->HasElement("angular_velocities_limits")) {
        double angular_velocities_limits =
            _sdf->Get<double>("angular_velocities_limits");
        m_angular_velocities_limits[_entity] = angular_velocities_limits;
    }
    else {
        m_angular_velocities_limits[_entity] = 0.5;
    }

    auto name_opt = _ecm->Component<gz::sim::components::Name>(_entity);
    gzmsg << "WaypointFollower::Load Loading " << name_opt->Data() << std::endl;

    m_velocities[_entity] = {0, 0};

    return true;
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
    // TODO: add here to read waypoints off file to move model based on file
}

void WaypointFollowerPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
}
//////////////////////////////////////////////////
void WaypointFollowerPlugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        std::bind(&WaypointFollowerPlugin::Load, this, _1, _2, &_ecm));

    double dt =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();

    for (auto &&temp : m_waypoints) {
        gz::sim::Entity _entity = temp.first;
        std::vector<gz::math::Vector2d> _waypoints = temp.second;

        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(_entity)->Data();

        double angle_to_goal = 0;
        double distance_to_goal;

        // Vessel first movement set orientation
        // to goal to avoid weird curves
        if (m_waypoint_state.find(_entity) == m_waypoint_state.end()) {
            m_waypoint_state[_entity] = 0;
            // angle to goal and changing atan2 convention to gz convention
            gz::math::Vector3d goal = {
                _waypoints[0].X(), _waypoints[0].Y(), pose.Z()};
            // Direction vector to the goal from the model.
            gz::math::Vector3d direction = goal - pose.Pos();
            gz::math::Vector3d directionLocalFrame =
                pose.Rot().RotateVectorReverse(direction);
            gz::math::Angle bearing(
                atan2(directionLocalFrame.Y(), directionLocalFrame.X()));
            bearing.Normalize();

            // Setting angle
            pose.Rot().SetFromEuler(
                pose.Roll(), pose.Pitch(), bearing.Radian());
            _ecm.SetComponentData<gz::sim::components::Pose>(_entity, pose);
        }

        gz::math::Vector2d current_goal = _waypoints[m_waypoint_state[_entity]];
        gz::math::Vector3d goal = {
            current_goal.X(), current_goal.Y(), pose.Z()};
        gz::math::Vector3d direction = goal - pose.Pos();
        gz::math::Vector3d directionLocalFrame =
            pose.Rot().RotateVectorReverse(direction);
        gz::math::Angle bearing(
            atan2(directionLocalFrame.Y(), directionLocalFrame.X()));
        bearing.Normalize();
        angle_to_goal = bearing.Radian();
        distance_to_goal = directionLocalFrame.Length();

        double current_velocity = m_velocities[_entity][0];
        double stopping_distance =
            std::pow(current_velocity, 2) / 2 / m_linear_accel_limit[_entity];

        // We will only allow vessel to move forward for now.

        /**
         * @brief Flag for linear velocity
         * 0: Slow down to min speed
         * 1: Fully stop
         * 2: Speed up
         *
         */
        int linear_veloctiy_flag = 0;
        if (stopping_distance >= distance_to_goal) {
            if (m_waypoint_state[_entity] == m_waypoints[_entity].size() - 1 &&
                !m_loop[_entity]) {
                linear_veloctiy_flag = 1;
            }
            else {
                linear_veloctiy_flag = 0;
            }
        }
        else if (angle_to_goal < 1.57 && angle_to_goal > -1.57) {
            linear_veloctiy_flag = 2;
        }
        double accel = m_linear_accel_limit[_entity] * dt / 1000;
        switch (linear_veloctiy_flag) {
        case 1: {
            if (m_velocities[_entity][0] < 0) {
                m_velocities[_entity][0] = 0;
            }
            else if (m_velocities[_entity][0] > 0) {
                m_velocities[_entity][0] -= accel;
            }
            break;
        }
        case 2: {
            // match to speed limit
            if (current_velocity + accel >
                m_linear_velocities_limits[_entity][1]) {
                m_velocities[_entity][0] =
                    m_linear_velocities_limits[_entity][1];
            }
            // full acceleartion
            else {
                m_velocities[_entity][0] += accel;
            }
            break;
        }
        default: {
            // Default case 0
            // If speed is less than min, continue;

            // if speed deduct is less than min, set to min

            // else deduct

            // match to speed limit
            if (current_velocity - accel <
                m_linear_velocities_limits[_entity][0]) {
                m_velocities[_entity][0] =
                    m_linear_velocities_limits[_entity][0];
            }
            // full acceleartion
            else {
                m_velocities[_entity][0] -= accel;
            }
            break;
        }
        }

        /*
        Handling orientation
        0: null
        1: accelerate left
        2: accelerate right
        3: zeroing
        */
        stopping_distance = std::pow(m_velocities[_entity][1], 2) / 2 /
                            m_angular_accel_limit[_entity];
        int orientation_accel_flag = 0;
        if ((angle_to_goal < 0.1 && angle_to_goal > -0.1) ||
            stopping_distance >= abs(angle_to_goal)) {
            orientation_accel_flag = 3;
        }
        else if (angle_to_goal >= 0.1) {
            orientation_accel_flag = 1;
        }
        else {
            orientation_accel_flag = 2;
        }

        accel = m_angular_accel_limit[_entity] * dt / 1000;
        switch (orientation_accel_flag) {
        case 1: {
            // Turning left, +ve angle
            if (m_velocities[_entity][1] + accel >=
                m_angular_velocities_limits[_entity]) {
                m_velocities[_entity][1] = m_angular_velocities_limits[_entity];
            }
            else {
                m_velocities[_entity][1] += accel;
            }
            break;
        }
        case 2: {
            // Turning right, -ve angle
            if (m_velocities[_entity][1] - accel <=
                -1 * m_angular_velocities_limits[_entity]) {
                m_velocities[_entity][1] =
                    -1 * m_angular_velocities_limits[_entity];
            }
            else {
                m_velocities[_entity][1] -= accel;
            }
            break;
        }
        case 3: {
            // Zeroing
            if (m_velocities[_entity][1] > 0) {
                // turning left
                if (m_velocities[_entity][1] - accel <= 0) {
                    m_velocities[_entity][1] = 0;
                }
                else {
                    m_velocities[_entity][1] -= accel;
                }
            }
            else {
                if (m_velocities[_entity][1] + accel >= 0) {
                    m_velocities[_entity][1] = 0;
                }
                else {
                    m_velocities[_entity][1] += accel;
                }
            }
        }
        default: {
            break;
        }
        }

        // Updating pose
        pose.SetX(
            pose.X() +
            (m_velocities[_entity][0] * cos(pose.Yaw()) * dt / 1000));
        pose.SetY(
            pose.Y() +
            (m_velocities[_entity][0] * sin(pose.Yaw()) * dt / 1000));
        pose.Rot().SetFromEuler(
            pose.Roll(),
            pose.Pitch(),
            pose.Yaw() + dt * m_velocities[_entity][1]);

        _ecm.SetComponentData<gz::sim::components::Pose>(_entity, pose);

        // goal tracking
        if (distance_to_goal <= m_rangeTolerance[_entity]) {
            // Last point
            if (m_waypoint_state[_entity] == m_waypoints[_entity].size() - 1) {
                // loop
                if (m_loop[_entity])
                    m_waypoint_state[_entity] = 0;
                // Exit
                else if (m_velocities[_entity][0] <= 0.1) {
                    m_waypoints.erase(_entity);
                    m_waypoint_state.erase(_entity);
                }
            }
            else {
                m_waypoint_state[_entity] += 1;
            }
        }
    }
}

} // namespace gazebo
} // namespace liquidai

GZ_ADD_PLUGIN(
    liquidai::gazebo::WaypointFollowerPlugin,
    gz::sim::System,
    liquidai::gazebo::WaypointFollowerPlugin::ISystemConfigure,
    liquidai::gazebo::WaypointFollowerPlugin::ISystemPreUpdate,
    liquidai::gazebo::WaypointFollowerPlugin::ISystemUpdate)
