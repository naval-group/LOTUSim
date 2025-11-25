/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

#include "waypoint_follower/waypoint_follower.hpp"

namespace lotusim::gazebo {
using namespace std::placeholders;

//////////////////////////////////////////////////
WaypointFollowerPlugin::WaypointFollowerPlugin()
{
    m_logger = logger::createConsoleAndFileLogger(
        "waypoing_follower",
        "waypoing_follower.txt");
}

//////////////////////////////////////////////////
WaypointFollowerPlugin::~WaypointFollowerPlugin()
{
    rclcpp::shutdown();
    m_ros_node_thread->join();
    m_logger->info(
        "EntityManager::~EntityManager: EntityManager successfully shutdown.");
}

//////////////////////////////////////////////////
bool WaypointFollowerPlugin::load(
    const gz::sim::Entity& _entity,
    sdf::ElementPtr lotus_param_sdf,
    gz::sim::EntityComponentManager& _ecm)
{
    auto name_opt = _ecm.Component<gz::sim::components::Name>(_entity);
    if (name_opt) {
        m_logger->info("WaypointFollower::load Loading {}", name_opt->Data());

        std::string model_name = name_opt->Data();

        // Create ros subscription for waypoints to override given waypoints
        m_subscription.push_back(
            m_ros_node->create_subscription<geographic_msgs::msg::GeoPath>(
                model_name + "/waypoints",
                10,
                [this,
                 _entity](const geographic_msgs::msg::GeoPath::SharedPtr msg) {
                    this->m_waypoints[_entity].clear();
                    for (auto const& geo_point_stamped : msg->poses) {
                        gz::math::Vector3d xyz =
                            m_origin_spherical.LocalFromSphericalPosition(
                                gz::math::Vector3d{
                                    geo_point_stamped.pose.position.latitude,
                                    geo_point_stamped.pose.position.longitude,
                                    0});
                        this->m_waypoints[_entity].push_back(
                            {xyz.X(), xyz.Y()});
                    }
                }));
    } else {
        m_logger->warn(
            "WaypointFollower::load Loading entity with no name: {}",
            _entity);
        return false;
    }

    sdf::ElementPtr _sdf;

    if (lotus_param_sdf->HasElement("follower")) {
        _sdf = lotus_param_sdf->GetElement("follower");
    } else {
        return true;
    }

    m_prevHeadingError[_entity] = 0.0;
    m_headingIntegral[_entity] = 0.0;

    auto pose_opt = _ecm.Component<gz::sim::components::Pose>(_entity);
    if (!pose_opt) {
        return false;
    }
    auto pose = pose_opt->Data();
    // Parse the optional <waypoints> element.
    std::vector<gz::math::Vector2d> waypoint;
    if (_sdf->HasElement("waypoints")) {
        auto waypointsElem = _sdf->GetElement("waypoints");

        // We need at least one waypoint
        if (!waypointsElem->HasElement("waypoint")) {
            m_logger->error(
                "TrajectoryFollower: Unable to find <waypoints><waypoint> element in SDF.");
            return true;
        }
        auto waypointElem = waypointsElem->GetElement("waypoint");
        while (waypointElem) {
            gz::math::Vector2d position =
                waypointElem->Get<gz::math::Vector2d>();

            // Save the position.
            waypoint.push_back(position);

            // Print some debugging messages
            m_logger->debug(
                "Waypoint, Local: X = {} Y = {}",
                position.X(),
                position.Y());

            waypointElem = waypointElem->GetNextElement("waypoint");
        }
    }
    // If no waypoints present, check for the <circle> element and parse.
    else if (_sdf->HasElement("circle")) {
        m_logger->debug("Circle element activated");
        auto circleElem = _sdf->GetElement("circle");

        if (!circleElem->HasElement("radius")) {
            m_logger->error("No <circle><radius> specified");
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
            auto new_position = position + vec;
            waypoint.push_back(new_position);
            angle += 2 * GZ_PI / 8;
            vec.Set(radius * cos(angle), radius * sin(angle));
            m_logger->debug(
                "Entered circle waypoint ({},{})",
                new_position.X(),
                new_position.Y());
        }
    }
    // If no waypoints or circle, check for the <line> element and parse.
    else if (_sdf->HasElement("line")) {
        auto lineElem = _sdf->GetElement("line");
        // Parse the required <direction> field.
        if (!lineElem->HasElement("direction")) {
            m_logger->error("No <line><direction> specified");
            return true;
        }
        gz::math::Angle direction = lineElem->Get<gz::math::Angle>("direction");

        // Parse the required <length> field.
        if (!lineElem->HasElement("length")) {
            m_logger->error("No <line><length> specified");
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
        m_logger->debug(
            "Entered line waypoints ({},{}), ({},{})",
            position.X(),
            position.Y(),
            p2D.X(),
            p2D.Y());
    }
    m_waypoints[_entity] = waypoint;

    // Parse the optional <loop> element.
    if (_sdf->HasElement("loop"))
        m_loop[_entity] = _sdf->Get<bool>("loop");

    // Parse the optional <range_tolerance> element.
    if (_sdf->HasElement("range_tolerance")) {
        double rangeTolerance = _sdf->Get<double>("range_tolerance");
        m_rangeTolerance[_entity] = rangeTolerance;
    } else {
        m_rangeTolerance[_entity] = 0.5;
    }

    if (_sdf->HasElement("linear_accel_limit")) {
        double linear_accel_limit = _sdf->Get<double>("linear_accel_limit");
        m_linear_accel_limit[_entity] = linear_accel_limit;
    } else {
        m_linear_accel_limit[_entity] = 0.5;
    }

    if (_sdf->HasElement("angular_accel_limit")) {
        double angular_accel_limit = _sdf->Get<double>("angular_accel_limit");
        m_angular_accel_limit[_entity] = angular_accel_limit;
    } else {
        m_angular_accel_limit[_entity] = 0.01;
    }

    if (_sdf->HasElement("linear_velocities_limits")) {
        m_linear_velocities_limits[_entity] =
            _sdf->Get<gz::math::Vector2d>("linear_velocities_limits");
    } else {
        m_linear_velocities_limits[_entity] = gz::math::Vector2d(1, 10);
    }

    if (_sdf->HasElement("angular_velocities_limits")) {
        double angular_velocities_limits =
            _sdf->Get<double>("angular_velocities_limits");
        m_angular_velocities_limits[_entity] = angular_velocities_limits;
    } else {
        m_angular_velocities_limits[_entity] = 0.05;
    }

    m_velocities[_entity] = {0, 0};

    return true;
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>&,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/)
{
    m_world_entity = _entity;
    m_world_name = lotusim::common::getWorldName(_ecm);
    m_ros_node = rclcpp::Node::make_shared("wapoint_follower", m_world_name);
    if (rclcpp::ok()) {
        m_executor =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_ros_node);
        m_ros_node_thread =
            std::make_shared<std::thread>([&]() { m_executor->spin(); });
    } else {
        m_logger->error(
            "WaypointFollowerPlugin::Configure: RCLCPP context shutdown.");
    }

    gz::math::Angle lat0, lon0;
    gz::sim::Entity worldEntity;
    _ecm.Each<gz::sim::components::Name, gz::sim::components::World>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Name*,
            const gz::sim::components::World*) -> bool {
            worldEntity = _entity;
            return true;
        });
    if (_ecm.Component<gz::sim::components::SphericalCoordinates>(
            worldEntity)) {
        auto sphComp =
            _ecm.Component<gz::sim::components::SphericalCoordinates>(
                worldEntity);
        lat0 = sphComp->Data().LatitudeReference();
        lon0 = sphComp->Data().LongitudeReference();
    }
    // Create SphericalCoordinates helper
    m_origin_spherical.SetLatitudeReference(lat0);
    m_origin_spherical.SetLongitudeReference(lon0);
    m_origin_spherical.SetElevationReference(0);

    m_logger->info(
        "WaypointFollowerPlugin::Configure: Entity Manager started.");
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        [this](
            const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf* _model) {
            if (!_model)
                return true;

            sdf::ElementPtr _sdf = _model->Data().Element();
            if (_sdf->GetIncludeElement()) {
                _sdf = _sdf->GetIncludeElement();
            }
            if (_sdf->HasElement("lotus_param") &&
                _sdf->GetElement("lotus_param")
                    ->HasElement("waypoint_follower")) {
                _sdf = _sdf->GetElement("lotus_param")
                           ->GetElement("waypoint_follower");
            } else {
                return true;
            }
            m_model_load_queue[_entity] = _sdf;
            return true;
        });

    for (auto it = m_model_load_queue.begin();
         it != m_model_load_queue.end();) {
        gz::sim::Entity entity = it->first;
        sdf::ElementPtr sdfElem = it->second;

        bool res = load(entity, sdfElem, _ecm);
        if (res) {
            it = m_model_load_queue.erase(it);
        } else {
            ++it;
        }
    }

    double dt =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();

    for (auto&& temp : m_waypoints) {
        gz::sim::Entity _entity = temp.first;
        std::vector<gz::math::Vector2d> _waypoints = temp.second;
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(_entity)->Data();
        double angle_to_goal = 0;
        double distance_to_goal;

        if (m_waypoint_state.find(_entity) == m_waypoint_state.end()) {
            m_waypoint_state[_entity] = 0;
        }

        gz::math::Vector2d current_goal = _waypoints[m_waypoint_state[_entity]];
        gz::math::Vector3d goal = {
            current_goal.X(),
            current_goal.Y(),
            pose.Z()};
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

        /**
         * @brief Bang Bang controller for linear velocity
         *
         * Flag for linear velocity
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
            } else {
                linear_veloctiy_flag = 0;
            }
        } else if (angle_to_goal < 1.57 && angle_to_goal > -1.57) {
            linear_veloctiy_flag = 2;
        }
        double vel_change = m_linear_accel_limit[_entity] * dt / 1000;
        switch (linear_veloctiy_flag) {
            case 1: {
                if (m_velocities[_entity][0] < 0) {
                    m_velocities[_entity][0] = 0;
                } else if (m_velocities[_entity][0] > 0) {
                    m_velocities[_entity][0] -= vel_change;
                }
                break;
            }
            case 2: {
                // match to speed limit
                if (current_velocity + vel_change >
                    m_linear_velocities_limits[_entity][1]) {
                    m_velocities[_entity][0] =
                        m_linear_velocities_limits[_entity][1];
                }
                // full acceleartion
                else {
                    m_velocities[_entity][0] += vel_change;
                }
                break;
            }
            default: {
                // Default case 0
                // If speed is less than min, continue;
                // if speed deduct is less than min, set to min
                // else deduct

                // match to speed limit
                if (current_velocity - vel_change <
                    m_linear_velocities_limits[_entity][0]) {
                    m_velocities[_entity][0] =
                        m_linear_velocities_limits[_entity][0];
                }
                // full acceleartion
                else {
                    m_velocities[_entity][0] -= vel_change;
                }
                break;
            }
        }

        /*
        PID angular velocity
        Handling orientation
        0: null
        1: accelerate left
        2: accelerate right
        3: zeroing
        */
        double Kp_heading = 0.8;
        double Ki_heading = 0.05;
        double Kd_heading = 0.4;

        // heading error in global frame
        double desired_yaw = atan2(goal.Y() - pose.Y(), goal.X() - pose.X());
        double heading_error = desired_yaw - pose.Yaw();

        // normalize to [-pi, pi]
        while (heading_error > M_PI)
            heading_error -= 2 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2 * M_PI;

        // dt in seconds
        double dt_s = dt / 1000.0;

        // integrate error
        m_headingIntegral[_entity] += heading_error * dt_s;

        // derivative term
        double d_error = (heading_error - m_prevHeadingError[_entity]) / dt_s;
        m_prevHeadingError[_entity] = heading_error;

        // PID output
        double desired_w = Kp_heading * heading_error +
                           Ki_heading * m_headingIntegral[_entity] +
                           Kd_heading * d_error;

        // clamp to angular velocity limits
        double max_w = m_angular_velocities_limits[_entity];
        if (desired_w > max_w)
            desired_w = max_w;
        if (desired_w < -max_w)
            desired_w = -max_w;

        // apply accel limit to current angular velocity
        vel_change = m_angular_accel_limit[_entity] * dt_s;
        if (m_velocities[_entity][1] < desired_w)
            m_velocities[_entity][1] =
                std::min(m_velocities[_entity][1] + vel_change, desired_w);
        else
            m_velocities[_entity][1] =
                std::max(m_velocities[_entity][1] - vel_change, desired_w);

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
            } else {
                m_waypoint_state[_entity] += 1;
            }
        }
    }
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::WaypointFollowerPlugin,
    gz::sim::System,
    lotusim::gazebo::WaypointFollowerPlugin::ISystemConfigure,
    lotusim::gazebo::WaypointFollowerPlugin::ISystemUpdate)
