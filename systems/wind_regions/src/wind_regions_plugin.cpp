/*
 * Copyright (c) 2026 IRL CROSSING
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "wind_regions/wind_regions_plugin.hpp"

#include <gz/plugin/Register.hh>

namespace lotusim::gazebo {
using namespace std::placeholders;

// Hardcoded to match WIND_TOPIC / the new region topic in the ROS-side `Wind`
// agent (wind.py) exactly — like the rest of aerialWorld, this plugin only
// ever runs in that one world, so there is no per-world templating to do.
namespace {
constexpr const char* kWindTopic = "/aerialWorld/wind";
constexpr const char* kWindRegionsTopic = "/aerialWorld/wind/regions";
}  // namespace

WindRegionsPlugin::WindRegionsPlugin()
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}

WindRegionsPlugin::~WindRegionsPlugin()
{
    if (m_ros_executor) {
        m_ros_executor->cancel();
    }
    if (m_ros_node_thread && m_ros_node_thread->joinable()) {
        m_ros_node_thread->join();
    }
    if (m_logger) {
        m_logger->info(
            "WindRegionsPlugin::~WindRegionsPlugin: successfully shutdown.");
    }
}

void WindRegionsPlugin::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&)
{
    m_world_name = lotusim::common::getWorldName(_ecm);
    m_logger = logger::createConsoleAndFileLogger(
        "wind_regions_plugin",
        m_world_name + "_wind_regions_plugin.txt");

    m_scaling_factor = _sdf->Get<double>("scaling_factor", 1.0).first;

    // Turbulence knobs. Every default here is the "off" value, so a world
    // file written before this feature existed keeps its exact previous
    // behaviour: sigma 0 everywhere makes the gust term identically zero.
    m_ambient_sigma =
        std::max(0.0, _sdf->Get<double>("ambient_turbulence_sigma", 0.0).first);
    m_turbulence_time_constant =
        std::max(1e-3, _sdf->Get<double>("turbulence_time_constant", 1.0).first);
    m_sigma_ratio_v =
        std::max(0.0, _sdf->Get<double>("turbulence_sigma_ratio_v", 0.8).first);
    m_sigma_ratio_w =
        std::max(0.0, _sdf->Get<double>("turbulence_sigma_ratio_w", 0.5).first);
    // A fixed default seed rather than a clock-derived one: an unseeded run
    // that silently differs from the last is the harder failure to notice,
    // and every experiment we run wants repeatability by default. Vary the
    // seed explicitly, per repetition, when independent draws are the point.
    const int seed = _sdf->Get<int>("turbulence_seed", 0).first;
    m_rng.seed(static_cast<std::mt19937::result_type>(seed));

    m_ros_node = rclcpp::Node::make_shared("wind_regions_plugin", m_world_name);

    // kWindTopic is written by both the Wind agent (TRANSIENT_LOCAL) and,
    // while the agent is passive, directly by the Unity sliders over
    // ROS-TCP — whose publisher QoS this plugin does not control. A VOLATILE
    // (plain, depth-10) subscription is compatible with either durability;
    // requesting TRANSIENT_LOCAL here would silently drop all messages if
    // the Unity-side publisher happens to be VOLATILE (DDS QoS
    // incompatibility yields zero delivery, not a fallback). Matches the
    // QoS the old wind_ros_to_gz_bridge used for the same reason.
    m_wind_sub = m_ros_node->create_subscription<lotusim_msgs::msg::Wind>(
        kWindTopic,
        rclcpp::QoS(10),
        [this](lotusim_msgs::msg::Wind::ConstSharedPtr msg) -> void {
            std::lock_guard<std::mutex> lock(m_wind_mutex);
            m_global_wind.Set(
                msg->linear_velocity.x,
                msg->linear_velocity.y,
                msg->linear_velocity.z);
            m_global_enable_wind = msg->enable_wind;
        });

    // kWindRegionsTopic, unlike kWindTopic, is only ever written by the Wind
    // agent — both ends are ours, so TRANSIENT_LOCAL is safe here and means
    // this plugin gets the current region list immediately on startup
    // instead of waiting for the agent's next periodic publish.
    m_regions_sub = m_ros_node->create_subscription<
        lotusim_msgs::msg::WindRegionArray>(
        kWindRegionsTopic,
        rclcpp::QoS(1).transient_local(),
        [this](lotusim_msgs::msg::WindRegionArray::ConstSharedPtr msg)
            -> void {
            std::vector<RegionState> regions;
            regions.reserve(msg->regions.size());
            for (auto&& region : msg->regions) {
                RegionState state;
                state.id = region.id;
                state.shape = MakeShape(region);
                state.velocity.Set(
                    region.linear_velocity.x,
                    region.linear_velocity.y,
                    region.linear_velocity.z);
                state.turbulence_sigma = std::max(0.0, region.turbulence_sigma);
                state.radial_speed.assign(
                    region.radial_speed.begin(), region.radial_speed.end());
                state.enable_wind = region.enable_wind;
                regions.push_back(std::move(state));
            }
            std::lock_guard<std::mutex> lock(m_wind_mutex);
            m_regions = std::move(regions);
        });

    // Spin on a dedicated thread so both subscriptions are drained on
    // arrival rather than at most one message per Update() (see
    // PhysicsInterfacePlugin's identical m_ros_executor for why spin_some()
    // is not enough here).
    if (rclcpp::ok()) {
        m_ros_executor =
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_ros_executor->add_node(m_ros_node);
        m_ros_node_thread = std::make_shared<std::thread>(
            [this]() { m_ros_executor->spin(); });
    } else {
        m_logger->error("WindRegionsPlugin::Configure: RCLCPP context shutdown.");
    }

    m_logger->info(
        "WindRegionsPlugin ready (scaling_factor={}, ambient_turbulence_sigma={} "
        "m/s, turbulence_time_constant={} s, sigma_ratio_v/w={}/{}, seed={}). "
        "Subscribed to {} and {}.",
        m_scaling_factor,
        m_ambient_sigma,
        m_turbulence_time_constant,
        m_sigma_ratio_v,
        m_sigma_ratio_w,
        seed,
        kWindTopic,
        kWindRegionsTopic);
}

std::shared_ptr<const WindRegionsPlugin::RegionShape> WindRegionsPlugin::MakeShape(
    const lotusim_msgs::msg::WindRegion& _region)
{
    if (_region.shape_type == lotusim_msgs::msg::WindRegion::CONE_SEGMENT) {
        return std::make_shared<ConeSegmentShape>(
            gz::math::Vector3d(
                _region.cone.origin.x, _region.cone.origin.y, _region.cone.origin.z),
            gz::math::Vector2d(_region.cone.axis.x, _region.cone.axis.y),
            _region.cone.length,
            _region.cone.r_start,
            _region.cone.r_end);
    }
    return std::make_shared<BoxShape>(
        _region.box.x1, _region.box.y1, _region.box.x2, _region.box.y2);
}

double WindRegionsPlugin::SampleRadial(
    const std::vector<double>& _profile,
    double _frac)
{
    // Samples are evenly spaced in r/r_boundary, index 0 on the axis and the
    // last index at the boundary, so the position in the array is just the
    // fraction scaled by the number of intervals. Linear between samples: the
    // point is to restore a continuous gradient, and anything higher-order
    // would be inventing structure the producer did not send.
    const std::size_t n = _profile.size();
    if (n == 0) {
        return 0.0;
    }
    if (n == 1) {
        return _profile[0];
    }
    const double t = std::clamp(_frac, 0.0, 1.0) * static_cast<double>(n - 1);
    const std::size_t i = static_cast<std::size_t>(t);
    if (i >= n - 1) {
        return _profile[n - 1];
    }
    return _profile[i] + (t - static_cast<double>(i)) * (_profile[i + 1] - _profile[i]);
}

bool WindRegionsPlugin::ResolveWind(
    double _x,
    double _y,
    double _z,
    const std::vector<RegionState>& _regions,
    const gz::math::Vector3d& _globalWind,
    bool _globalEnableWind,
    double _ambientSigma,
    gz::math::Vector3d& _wind,
    double& _sigma)
{
    // Last matching region in the list wins on overlap.
    double frac = 0.0;
    for (auto it = _regions.rbegin(); it != _regions.rend(); ++it) {
        if (it->Contains(_x, _y, _z, &frac)) {
            if (!it->enable_wind) {
                return false;
            }
            _wind = it->velocity;
            // A graded region carries speed sampled across its radius. Without
            // this the whole cross-section gets linear_velocity, which for a
            // wake means every point inside is handed the centreline deficit.
            const auto& prof = it->radial_speed;
            if (prof.size() >= 2) {
                const double u = SampleRadial(prof, frac);
                const double len = _wind.Length();
                // Direction is carried by linear_velocity; only magnitude
                // varies with radius. A zero vector has no direction to
                // scale, so leave it alone rather than divide by zero.
                if (len > 1e-9) {
                    _wind *= (u / len);
                }
            }
            _sigma = it->turbulence_sigma;
            return true;
        }
    }
    if (!_globalEnableWind) {
        return false;
    }
    _wind = _globalWind;
    _sigma = _ambientSigma;
    return true;
}

gz::math::Vector3d WindRegionsPlugin::AdvanceGust(
    gz::sim::Entity _entity,
    double _dt,
    double _sigma,
    const gz::math::Vector3d& _wind)
{
    if (_sigma <= 0.0 || _dt <= 0.0) {
        return gz::math::Vector3d::Zero;
    }

    // Exact discrete Ornstein-Uhlenbeck update, held at unit variance:
    //   a = exp(-dt/T)              how much of the previous gust survives
    //   b = sqrt(1 - a*a)           the fresh noise that keeps Var(n) == 1
    // a^2 + b^2 == 1 is what makes this exact for any dt rather than a
    // small-dt approximation, so the gust statistics do not drift if the
    // physics step ever changes.
    const double a = std::exp(-_dt / m_turbulence_time_constant);
    const double b = std::sqrt(std::max(0.0, 1.0 - a * a));

    gz::math::Vector3d& n = m_gust_state[_entity];
    n.Set(
        a * n.X() + b * m_normal(m_rng),
        a * n.Y() + b * m_normal(m_rng),
        a * n.Z() + b * m_normal(m_rng));

    // Resolve the three components onto the local mean wind: along it,
    // horizontally across it, and vertically. Atmospheric turbulence is not
    // isotropic — the vertical component is roughly half the along-wind one
    // (IEC 61400-1 normal turbulence model) — and a multirotor does not
    // respond the same way to a headwind gust as to a downdraft, so the
    // distinction is worth keeping.
    gz::math::Vector3d along(_wind.X(), _wind.Y(), 0.0);
    const double speed = along.Length();
    if (speed < 1e-6) {
        // Calm mean wind: no preferred horizontal axis to resolve onto, so
        // fall back to world axes rather than an arbitrary one.
        along.Set(1.0, 0.0, 0.0);
    } else {
        along /= speed;
    }
    const gz::math::Vector3d across(-along.Y(), along.X(), 0.0);

    return along * (_sigma * n.X()) + across * (_sigma * m_sigma_ratio_v * n.Y())
           + gz::math::Vector3d::UnitZ * (_sigma * m_sigma_ratio_w * n.Z());
}

void WindRegionsPlugin::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    if (_info.paused) {
        return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();

    // Newly spawned wind-enabled links need velocity checks turned on once
    // before WorldLinearVelocity() will return a value (see Link::Enable
    // VelocityChecks docs) — mirrors PhysicsInterfacePlugin::loadVessel.
    _ecm.EachNew<gz::sim::components::Link, gz::sim::components::WindMode>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Link*,
            const gz::sim::components::WindMode*) -> bool {
            gz::sim::Link(_entity).EnableVelocityChecks(_ecm);
            return true;
        });

    // Drop the turbulence state of links that have gone away, so a scenario
    // that spawns and despawns vehicles does not accumulate dead entries.
    _ecm.EachRemoved<gz::sim::components::Link>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Link*) -> bool {
            m_gust_state.erase(_entity);
            return true;
        });

    std::vector<RegionState> regions_snapshot;
    gz::math::Vector3d global_wind_snapshot;
    bool global_enable_snapshot;
    {
        std::lock_guard<std::mutex> lock(m_wind_mutex);
        regions_snapshot = m_regions;
        global_wind_snapshot = m_global_wind;
        global_enable_snapshot = m_global_enable_wind;
    }

    // Diagnostic counters for the throttled summary below — cheap, and the
    // only way to tell "no wind-enabled link found" from "found but missing
    // pose/velocity/inertial" from "force applied" without attaching a
    // debugger.
    int links_seen = 0;
    int links_skipped_no_data = 0;
    int links_force_applied = 0;
    gz::math::Vector3d last_force;
    gz::math::Vector3d last_wind;
    double last_sigma = 0.0;

    _ecm.Each<gz::sim::components::Link, gz::sim::components::WindMode>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Link*,
            const gz::sim::components::WindMode* _windMode) -> bool {
            if (!_windMode->Data()) {
                return true;
            }
            ++links_seen;

            gz::sim::Link link(_entity);
            auto pose = link.WorldPose(_ecm);
            auto lin_vel = link.WorldLinearVelocity(_ecm);
            auto inertial = link.WorldInertial(_ecm);
            if (!pose || !lin_vel || !inertial) {
                ++links_skipped_no_data;
                return true;
            }

            gz::math::Vector3d wind;
            double sigma = 0.0;
            if (!ResolveWind(
                    pose->Pos().X(),
                    pose->Pos().Y(),
                    pose->Pos().Z(),
                    regions_snapshot,
                    global_wind_snapshot,
                    global_enable_snapshot,
                    m_ambient_sigma,
                    wind,
                    sigma)) {
                return true;
            }

            // The turbulent fluctuation rides on top of the mean field: it is
            // zero-mean by construction, so it perturbs what this link feels
            // without shifting the mean wind the wake model was validated
            // against. Deliberately applied only here, on the vehicle force
            // path — the rotor-averaged speeds that drive turbine power, and
            // the wake geometry itself, both stay on the mean field.
            wind += AdvanceGust(_entity, dt, sigma, wind);

            // Matches stock WindEffects: force = mass * scaling_factor *
            // (wind - link_velocity), with `wind` now carrying the gust
            // added above, so mass cancels out of the resulting
            // acceleration (a = scaling_factor * relative_velocity,
            // independent of the link's mass) — without the mass factor a
            // heavier link (e.g. x500's 2 kg base_link) would visibly
            // under-react compared to the stock plugin's behaviour.
            const double mass = inertial->MassMatrix().Mass();
            const gz::math::Vector3d force =
                mass * m_scaling_factor * (wind - lin_vel.value());
            link.AddWorldForce(_ecm, force);
            ++links_force_applied;
            last_force = force;
            last_wind = wind;
            last_sigma = sigma;
            return true;
        });

    // Roughly once a second (world runs physics at 500 Hz per aerialWorld.world).
    if (++m_update_count % 500 == 0) {
        m_logger->debug(
            "WindRegionsPlugin::Update: {} wind-enabled link(s), {} skipped "
            "(missing pose/velocity/inertial), {} with force applied (last: "
            "wind=[{:.2f},{:.2f},{:.2f}] sigma={:.2f} "
            "force=[{:.2f},{:.2f},{:.2f}]N).",
            links_seen,
            links_skipped_no_data,
            links_force_applied,
            last_wind.X(),
            last_wind.Y(),
            last_wind.Z(),
            last_sigma,
            last_force.X(),
            last_force.Y(),
            last_force.Z());
    }
}

}  // namespace lotusim::gazebo
GZ_ADD_PLUGIN(
    lotusim::gazebo::WindRegionsPlugin,
    gz::sim::System,
    lotusim::gazebo::WindRegionsPlugin::ISystemConfigure,
    lotusim::gazebo::WindRegionsPlugin::ISystemUpdate)
