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
        "WaypointFollowerPlugin::~WaypointFollowerPlugin: WaypointFollowerPlugin successfully shutdown.");
}

//////////////////////////////////////////////////
bool WaypointFollowerPlugin::load(
    const gz::sim::Entity& _entity,
    sdf::ElementPtr lotus_param_sdf,
    gz::sim::EntityComponentManager& _ecm)
{
    auto name_opt = _ecm.Component<gz::sim::components::Name>(_entity);

    if (!name_opt) {
        m_logger->warn(
            "WaypointFollower::load Loading entity with no name: {}",
            _entity);
        return false;
    }
    std::string model_name = name_opt->Data();

    m_logger->info("WaypointFollower::load Loading {}", model_name);

    if (!lotus_param_sdf) {
        return true;  // No SDF, keep defaults
    }

    // Always do the ROS-topic initialization regardless of lotus_param_sdf
    setupRosForModel(_entity, model_name);

    // Initialize default values regardless of SDF
    m_prev_heading_error[_entity] = 0.0;
    m_heading_integral[_entity] = 0.0;
    m_waypoints[_entity] = {};
    m_loop[_entity] = false;
    m_rangeTolerance[_entity] = 0.5;
    m_linear_accel_limit[_entity] = 0.5;
    m_angular_accel_limit[_entity] = 0.01;
    m_linear_velocities_limits[_entity] = gz::math::Vector2d(1, 10);
    m_angular_velocities_limits[_entity] = 0.05;
    m_velocities[_entity] = {0, 0};
    m_linear_pid[_entity] = {0.5, 0.05, 0.1};
    m_angular_pid[_entity] = {0.8, 0.05, 0.4};
    m_prev_yaw[_entity] = 0.0;

    sdf::ElementPtr _sdf;

    if (lotus_param_sdf->HasElement("follower")) {
        _sdf = lotus_param_sdf->GetElement("follower");
    } else {
        return true;  // No follower element, defaults are enough
    }

    auto pose_opt = _ecm.Component<gz::sim::components::Pose>(_entity);
    if (!pose_opt) {
        return false;
    }
    // Parse the optional <waypoints> element.
    auto pose = pose_opt->Data();
    std::vector<gz::math::Vector2d> waypoint;
    if (_sdf->HasElement("waypoints")) {
        auto waypointsElem = _sdf->GetElement("waypoints");

        // We need at least one waypoint
        if (!waypointsElem->HasElement("waypoint")) {
            m_logger->error(
                "WaypointFollowerPlugin::Load:: Unable to find <waypoints><waypoint> element in SDF.");
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
                "WaypointFollowerPlugin::Load: Waypoint, Local: X = {} Y = {}",
                position.X(),
                position.Y());
            waypointElem = waypointElem->GetNextElement("waypoint");
        }
    }
    // If no waypoints present, check for the <circle> element and parse.
    else if (_sdf->HasElement("circle")) {
        m_logger->debug(
            "WaypointFollowerPlugin::Load: Circle element activated");
        auto circleElem = _sdf->GetElement("circle");

        if (!circleElem->HasElement("radius")) {
            m_logger->error(
                "WaypointFollowerPlugin::Load: No <circle><radius> specified");
            return true;
        }

        // Parse the required <radius> field.
        double radius = circleElem->Get<double>("radius");

        // Get the current model position in global coordinates.
        // Create local vectors that represent a path along a rough circle.
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
                "WaypointFollowerPlugin::Load: Entered circle waypoint ({},{})",
                new_position.X(),
                new_position.Y());
        }
    }
    // If no waypoints or circle, check for the <line> element and parse.
    else if (_sdf->HasElement("line")) {
        auto lineElem = _sdf->GetElement("line");
        // Parse the required <direction> field.
        if (!lineElem->HasElement("direction")) {
            m_logger->error(
                "WaypointFollowerPlugin::Load: No <line><direction> specified");
            return true;
        }
        gz::math::Angle direction = lineElem->Get<gz::math::Angle>("direction");

        // Parse the required <length> field.
        if (!lineElem->HasElement("length")) {
            m_logger->error(
                "WaypointFollowerPlugin::Load: No <line><length> specified");
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
            "WaypointFollowerPlugin::Load: Entered line waypoints ({},{}), ({},{})",
            position.X(),
            position.Y(),
            p2D.X(),
            p2D.Y());
    }
    {
        std::lock_guard<std::mutex> lock(m_waypoint_mutex);

        m_waypoints[_entity] = waypoint;

        for (auto&& point : waypoint) {
            gz::math::Vector3d latlong =
                m_origin_spherical.SphericalFromLocalPosition(
                    {point.X(), point.Y(), 0});
            geographic_msgs::msg::GeoPoint point_msg;
            point_msg.latitude = latlong.X();
            point_msg.longitude = latlong.Y();

            m_waypoints_geo[_entity].push_back(point_msg);
        }
    }
    // Optional overrides from SDF
    if (_sdf->HasElement("loop"))
        m_loop[_entity] = _sdf->Get<bool>("loop");

    if (_sdf->HasElement("range_tolerance"))
        m_rangeTolerance[_entity] = _sdf->Get<double>("range_tolerance");

    if (_sdf->HasElement("linear_accel_limit"))
        m_linear_accel_limit[_entity] = _sdf->Get<double>("linear_accel_limit");

    if (_sdf->HasElement("angular_accel_limit"))
        m_angular_accel_limit[_entity] =
            _sdf->Get<double>("angular_accel_limit");

    if (_sdf->HasElement("linear_velocities_limits"))
        m_linear_velocities_limits[_entity] =
            _sdf->Get<gz::math::Vector2d>("linear_velocities_limits");

    if (_sdf->HasElement("angular_velocities_limits"))
        m_angular_velocities_limits[_entity] =
            _sdf->Get<double>("angular_velocities_limits");

    if (_sdf->HasElement("linear_pid")) {
        auto pid = _sdf->Get<gz::math::Vector3d>("linear_pid");
        m_linear_pid[_entity][0] = pid.X();
        m_linear_pid[_entity][1] = pid.Y();
        m_linear_pid[_entity][2] = pid.Z();
    };

    if (_sdf->HasElement("angular_pid")) {
        auto pid = _sdf->Get<gz::math::Vector3d>("angular_pid");
        m_angular_pid[_entity][0] = pid.X();
        m_angular_pid[_entity][1] = pid.Y();
        m_angular_pid[_entity][2] = pid.Z();
    };
    return true;
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/)
{
    m_world_entity = _entity;
    m_world_name = lotusim::common::getWorldName(_ecm);

    m_status_update_period = 10;
    auto sdfPtr = const_cast<sdf::Element*>(_sdf.get());
    if (sdfPtr->HasElement("update_rate")) {
        m_status_update_period = 1 / sdfPtr->Get<float>("update_rate");
        m_logger->info(
            "WaypointFollowerPlugin::Configure: update period set to  {}s",
            m_status_update_period);
    } else {
        m_logger->warn(
            "WaypointFollowerPlugin::Configure: Status publishing rate not found. Setting to default 10s");
        return;
    }

    // Create ROS node
    m_ros_node = rclcpp::Node::make_shared("waypoint_follower", m_world_name);
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

    // Get spherical coordinates
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
        [this, &_ecm](
            const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf* _model) {
            if (!_model)
                return true;

            sdf::ElementPtr lotus_param_sdf = nullptr;

            sdf::ElementPtr _sdf = _model->Data().Element();
            if (_sdf->GetIncludeElement()) {
                _sdf = _sdf->GetIncludeElement();
            }
            if (_sdf->HasElement("lotus_param") &&
                _sdf->GetElement("lotus_param")
                    ->HasElement("waypoint_follower")) {
                lotus_param_sdf = _sdf->GetElement("lotus_param")
                                      ->GetElement("waypoint_follower");
            }

            m_model_load_queue[_entity] = lotus_param_sdf;

            auto name_comp = _ecm.Component<gz::sim::components::Name>(_entity);
            m_vessel_name[_entity] = name_comp ? name_comp->Data() : "unknown";
            return true;
        });

    for (auto it = m_model_load_queue.begin();
         it != m_model_load_queue.end();) {
        gz::sim::Entity entity = it->first;
        sdf::ElementPtr lotus_param_sdf = it->second;  // may be nullptr

        bool res = load(entity, lotus_param_sdf, _ecm);
        if (res) {
            it = m_model_load_queue.erase(it);
        } else {
            ++it;
        }
    }

    double dt =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();
    // dt in seconds
    double dt_s = dt / 1000.0;

    for (auto&& temp : m_waypoints) {
        gz::sim::Entity _entity = temp.first;
        std::vector<gz::math::Vector2d> _waypoints = temp.second;

        // Skip entities without Pose component
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
        if (!poseComp)
            continue;

        gz::math::Pose3d pose = poseComp->Data();

        // Skip if no waypoints
        if (_waypoints.empty())
            continue;

        // Initialize waypoint state if missing
        if (m_waypoint_state.find(_entity) == m_waypoint_state.end()) {
            m_waypoint_state[_entity] = 0;
        }

        // Clamp waypoint index
        if (m_waypoint_state[_entity] >= _waypoints.size()) {
            m_waypoint_state[_entity] = 0;
        }

        double angle_to_goal = 0;
        double distance_to_goal;
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

        // --- DEBUG ---
        size_t current_index = m_waypoint_state[_entity];

        double current_velocity = m_velocities[_entity][0];
        bool is_last_waypoint =
            (m_waypoint_state[_entity] == m_waypoints[_entity].size() - 1) &&
            !m_loop[_entity];

        // PID controller for linear velocity based on distance error
        double distance_error = distance_to_goal;

        // Integral term with anti-windup
        m_distance_error_integral[_entity] += distance_error * (dt_s);
        double max_integral_contribution =
            0.2 * m_linear_velocities_limits[_entity][1];
        double max_integral =
            max_integral_contribution / m_linear_pid[_entity][1];

        m_distance_error_integral[_entity] = std::clamp(
            m_distance_error_integral[_entity],
            -max_integral,
            max_integral);

        // Derivative term (note: derivative of distance error is -velocity)
        double distance_error_derivative =
            (distance_error - m_distance_error_previous[_entity]) / (dt_s);
        m_distance_error_previous[_entity] = distance_error;

        // PID gains (tune these values for your system)
        double kp = 0.5;   // Proportional gain (velocity per meter)
        double ki = 0.05;  // Integral gain
        double kd = 0.1;   // Derivative gain

        // Calculate desired velocity from PID
        double desired_velocity = kp * distance_error +
                                  ki * m_distance_error_integral[_entity] +
                                  kd * distance_error_derivative;

        // Modify based on angle to goal (reduce speed when turning)
        double angle_factor = std::cos(angle_to_goal);
        if (angle_factor < 0)
            angle_factor = 0;  // Don't go backwards
        desired_velocity *= angle_factor;

        // Apply velocity limits
        desired_velocity = std::clamp(
            desired_velocity,
            m_linear_velocities_limits[_entity][0],
            m_linear_velocities_limits[_entity][1]);

        // Special handling for last waypoint
        if (is_last_waypoint && distance_to_goal < 0.1) {
            desired_velocity = 0.0;
        }

        // Calculate velocity change with acceleration limits
        double velocity_change = desired_velocity - current_velocity;
        double max_accel = m_linear_accel_limit[_entity] * dt_s;
        velocity_change = std::clamp(velocity_change, -max_accel, max_accel);

        // Update velocity
        m_velocities[_entity][0] = current_velocity + velocity_change;

        // Final clamp to ensure limits
        m_velocities[_entity][0] = std::clamp(
            m_velocities[_entity][0],
            m_linear_velocities_limits[_entity][0],
            m_linear_velocities_limits[_entity][1]);

        /*
        PID angular velocity
        */
        double Kp_heading = m_angular_pid[_entity][0];
        double Ki_heading = m_angular_pid[_entity][1];
        double Kd_heading = m_angular_pid[_entity][2];

        double heading_error = angle_to_goal;
        double max_w = m_angular_velocities_limits[_entity];
        // Check for goal change to reset integral
        gz::math::Vector2d current_goal_2d(goal.X(), goal.Y());

        if (m_prev_yaw.find(_entity) == m_prev_yaw.end()) {
            m_prev_yaw[_entity] = pose.Yaw();
        }

        // integrate error
        max_integral_contribution = 0.2 * m_angular_velocities_limits[_entity];
        m_heading_integral[_entity] += heading_error * dt_s;

        // Clamp integral (avoid division by zero)
        if (Ki_heading > 1e-6) {
            double integral_max = max_integral_contribution / Ki_heading;
            m_heading_integral[_entity] = std::clamp(
                m_heading_integral[_entity],
                -integral_max,
                integral_max);
        }

        // Derivative term (on measurement to avoid kick)
        double yaw_diff = pose.Yaw() - m_prev_yaw[_entity];
        // Handling the wrapping of yaw when goal is behind the vessel
        while (yaw_diff > M_PI)
            yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI)
            yaw_diff += 2.0 * M_PI;
        double current_yaw_rate = yaw_diff / dt_s;

        m_prev_yaw[_entity] = pose.Yaw();
        double derivative_term = -Kd_heading * current_yaw_rate;

        // PID output
        double desired_w = Kp_heading * heading_error +
                           Ki_heading * m_heading_integral[_entity] +
                           derivative_term;

        // clamp to angular velocity limits
        double unclamped_w = desired_w;
        desired_w = std::clamp(desired_w, -max_w, max_w);

        // Anti-windup: back-calculate integral if output saturated
        if (Ki_heading > 1e-6 && std::abs(unclamped_w) > max_w) {
            double clamped_integral =
                (desired_w - Kp_heading * heading_error - derivative_term) /
                Ki_heading;
            m_heading_integral[_entity] = clamped_integral;
        }

        // Decay of I for anti-windup
        if (std::abs(heading_error) < 0.05) {
            m_heading_integral[_entity] *= 0.95;
        }

        // apply accel limit to current angular velocity
        double vel_change = m_angular_accel_limit[_entity] * dt_s;
        if (m_velocities[_entity][1] < desired_w) {
            m_velocities[_entity][1] =
                std::min(m_velocities[_entity][1] + vel_change, desired_w);
        } else {
            m_velocities[_entity][1] =
                std::max(m_velocities[_entity][1] - vel_change, desired_w);
        }

        // Updating pose
        pose.SetX(
            pose.X() + (m_velocities[_entity][0] * cos(pose.Yaw()) * dt_s));
        pose.SetY(
            pose.Y() + (m_velocities[_entity][0] * sin(pose.Yaw()) * dt_s));
        pose.Rot().SetFromEuler(
            pose.Roll(),
            pose.Pitch(),
            pose.Yaw() + dt_s * m_velocities[_entity][1]);

        _ecm.SetComponentData<gz::sim::components::Pose>(_entity, pose);

        m_logger->debug(
            "\n\n\n-------------------------------\n"
            "Entity {} is heading to waypoint {}/{}\n"
            "Position : {:.2f} {:.2f} Goal {:.2f} {:.2f}\n"
            "Distance to goal = {:.2f} Velocity = {:.2f}\n"
            "DirectionLocalFrame:X : {:.2f} Y : {:.2f}\n"
            "Yaw : {:.2f} Heading to goal = {:.2f} Angular Velocity = {:.2f} \n"
            "heading int: {:.2f}, yaw_rate: {:.2f}",
            _entity,
            current_index,
            _waypoints.size() - 1,
            pose.X(),
            pose.Y(),
            current_goal.X(),
            current_goal.Y(),
            distance_to_goal,
            m_velocities[_entity][0],
            directionLocalFrame.X(),
            directionLocalFrame.Y(),
            pose.Yaw(),
            angle_to_goal,
            m_velocities[_entity][1],
            m_heading_integral[_entity],
            current_yaw_rate);

        // Goal Tracking ----
        if (distance_to_goal <= m_rangeTolerance[_entity]) {
            m_heading_integral[_entity] = 0.0;
            m_distance_error_integral[_entity] = 0.0;
            bool isLast =
                (m_waypoint_state[_entity] == m_waypoints[_entity].size() - 1);

            publishStatus(_entity);

            if (isLast) {
                if (m_loop[_entity]) {
                    // loop back to the first waypoint
                    m_waypoint_state[_entity] = 0;
                } else {
                    // stop movement
                    m_velocities[_entity][0] = 0;

                    // clear after finishing all calculations and before next
                    // iteration
                    m_entities_to_remove.insert(_entity);

                    // skip the rest of update for this entity
                    continue;
                }
            } else {
                m_waypoint_state[_entity] += 1;
            }
        }
    }

    // Remove safely after the main update loop
    for (auto e : m_entities_to_remove) {
        stopVessel(e);
    }
    m_entities_to_remove.clear();
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::setupRosForModel(
    const gz::sim::Entity& entity,
    const std::string& model_name)
{
    m_waypoint_services.push_back(m_ros_node->create_service<
                                  lotusim_msgs::srv::SetWaypoints>(
        model_name + "/waypoints",
        [this, entity](
            const std::shared_ptr<lotusim_msgs::srv::SetWaypoints::Request>
                request,
            std::shared_ptr<lotusim_msgs::srv::SetWaypoints::Response>
                response) {
            try {
                std::lock_guard<std::mutex> lock(m_waypoint_mutex);
                m_waypoints_geo[entity] = request->path;
                m_waypoints[entity].clear();
                m_waypoint_state[entity] = 0;
                m_loop[entity] = request->loop;

                for (const auto& geo_point : request->path) {
                    gz::math::Vector3d xyz =
                        m_origin_spherical.LocalFromSphericalPosition(
                            {geo_point.latitude, geo_point.longitude, 0});

                    m_waypoints[entity].push_back({xyz.X(), xyz.Y()});
                }
                response->success = true;
            } catch (const std::exception& e) {
                m_logger->error(
                    "WaypointFollowerPlugin::SetWaypointCB:: error in setting new waypoint. {}.",
                    e.what());
            }
        }));

    m_waypoint_stop_services.push_back(
        m_ros_node->create_service<std_srvs::srv::Empty>(
            model_name + "/stop",
            [this, entity](
                const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                stopVessel(entity);
            }));

    // Create publisher for waypoint reached messages
    m_waypoint_pub[entity] =
        m_ros_node->create_publisher<lotusim_msgs::msg::WaypointFollowerStatus>(
            model_name + "/waypoint_reached",
            10);

    m_status_timer = m_ros_node->create_wall_timer(
        std::chrono::seconds(m_status_update_period),
        [this]() {
            for (const auto& [entity, publisher] : m_waypoint_pub) {
                publishStatus(entity);
            }
        });
}

//////////////////////////////////////////////////
bool WaypointFollowerPlugin::stopVessel(const gz::sim::Entity& entity)
{
    std::lock_guard<std::mutex> lock(m_waypoint_mutex);
    m_velocities[entity] = {0, 0};
    m_waypoints.erase(entity);
    m_waypoints_geo.erase(entity);
    m_waypoint_state.erase(entity);
    m_prev_heading_error.erase(entity);
    m_heading_integral.erase(entity);
    return true;
}

//////////////////////////////////////////////////
void WaypointFollowerPlugin::publishStatus(const gz::sim::Entity& _entity)
{
    std::lock_guard<std::mutex> lock(m_waypoint_mutex);
    auto it_pub = m_waypoint_pub.find(_entity);
    if (it_pub != m_waypoint_pub.end() && it_pub->second) {
        auto msg =
            std::make_shared<lotusim_msgs::msg::WaypointFollowerStatus>();
        msg->vessel_name = m_vessel_name[_entity];
        auto state_it = m_waypoint_state.find(_entity);
        auto wp_it = m_waypoints.find(_entity);
        auto geo_it = m_waypoints_geo.find(_entity);

        if (state_it != m_waypoint_state.end() && wp_it != m_waypoints.end() &&
            geo_it != m_waypoints_geo.end()) {
            const uint idx = state_it->second;

            if (idx < wp_it->second.size() && idx < geo_it->second.size()) {
                msg->waypoint_index = idx;
                msg->path = geo_it->second;
            }
        } else {
            msg->waypoint_index = 0;
            m_logger->warn(
                "WaypointFollowerPlugin::publishStatus: Waypoint index out of range for entity {}",
                _entity);
        }
        it_pub->second->publish(*msg);
    } else {
        m_logger->warn(
            "WaypointFollowerPlugin::publishStatus: Publisher not found for entity {}",
            _entity);
    }
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::WaypointFollowerPlugin,
    gz::sim::System,
    lotusim::gazebo::WaypointFollowerPlugin::ISystemConfigure,
    lotusim::gazebo::WaypointFollowerPlugin::ISystemUpdate)
