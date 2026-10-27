/*
 * Copyright (c) 2026 IRL CROSSING
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_GAZEBO_WIND_REGIONS_PLUGIN_HPP_
#define LOTUSIM_GAZEBO_WIND_REGIONS_PLUGIN_HPP_

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/WindMode.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"
#include "lotusim_msgs/msg/wind.hpp"
#include "lotusim_msgs/msg/wind_region.hpp"
#include "lotusim_msgs/msg/wind_region_array.hpp"

namespace lotusim::gazebo {

/**
 * @brief Replacement for gz-sim-wind-effects-system that supports multiple
 * 2D wind regions on top of a single ambient (global) wind vector.
 *
 * Every link tagged wind-enabled (`<enable_wind>true</enable_wind>`, same
 * convention as stock WindEffects — see components::WindMode) is, each
 * Update(), tested against the configured regions by its world X/Y position
 * (regions have no altitude bound, matching the ambient wind which has none
 * either). Each region owns a polymorphic RegionShape (BoxShape or
 * ConeSegmentShape today — see MakeShape) — the last region in the list
 * whose shape contains the link wins; a link outside every region falls
 * back to the ambient/global vector. The resolved vector produces a force
 * `mass * scaling_factor * (wind - link_velocity)`, applied via
 * Link::AddWorldForce.
 *
 * State of the art / prior art: the force formula above (including the
 * mass multiplication, which keeps the resulting acceleration independent of
 * a link's mass) is not original — it is gz-sim's own stock WindEffects
 * system's approximation (`force_approximation_scaling_factor`, see
 * gz-sim's `src/systems/wind_effects/WindEffects.cc`, upstream
 * https://github.com/gazebosim/gz-sim), reused as-is for physical/tuning
 * consistency with the plugin it replaces. What is original here: stock
 * WindEffects only ever resolves ONE wind vector for the whole world (its
 * own region concept is a *static*, SDF-only piecewise scalar multiplier on
 * that single vector — it cannot change wind *direction* by location, and
 * cannot be reconfigured without reloading the world). This plugin instead
 * resolves an arbitrary, runtime-defined list of 2D regions — each with its
 * own independent vector — received live over ROS, plus embeds its own ROS
 * node and subscribes directly to ROS topics instead of going through a
 * gz-transport bridge:
 *  - `/aerialWorld/wind` (lotusim_msgs/Wind) — ambient/global vector.
 *  - `/aerialWorld/wind/regions` (lotusim_msgs/WindRegionArray) — region list.
 *
 * Plugin format:
 *
 * <plugin filename="wind_regions_plugin"
 * name="lotusim::gazebo::WindRegionsPlugin">
 *   <scaling_factor>1.0</scaling_factor>
 *   <!-- All optional; the defaults shown leave turbulence disabled. -->
 *   <ambient_turbulence_sigma>0.0</ambient_turbulence_sigma>
 *   <turbulence_time_constant>1.0</turbulence_time_constant>
 *   <turbulence_sigma_ratio_v>0.8</turbulence_sigma_ratio_v>
 *   <turbulence_sigma_ratio_w>0.5</turbulence_sigma_ratio_w>
 *   <turbulence_seed>0</turbulence_seed>
 * </plugin>
 *
 * Ambient sigma is an SDF parameter rather than a field on lotusim_msgs/Wind
 * on purpose: /aerialWorld/wind is also published by Unity over ROS-TCP with
 * its own generated C# message class, so widening Wind.msg would break that
 * publisher until Unity's messages are regenerated in lockstep. Ambient
 * turbulence is a scenario constant anyway and does not need to ride a 10 Hz
 * topic. WindRegionArray has no such constraint — both ends of that topic are
 * ours — so per-region sigma does travel on the wire.
 */
class WindRegionsPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemUpdate {
public:
    WindRegionsPlugin();
    ~WindRegionsPlugin() override;

    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr) override;

    void Update(
        const gz::sim::UpdateInfo& _info,
        gz::sim::EntityComponentManager& _ecm) override;

private:
    /**
     * @brief One region's geometry, abstracted behind a single virtual
     * Contains() so callers (RegionState, ResolveWind) never branch on which
     * shape a region actually is. Adding a new shape means adding a new
     * subclass and one line in MakeShape(), rather than a shape-type branch at
     * every call site.
     */
    class RegionShape {
    public:
        virtual ~RegionShape() = default;
        /// @param _frac if non-null, receives the query point's radial
        /// position in the region, 0 on the axis and 1 at the boundary. Only
        /// meaningful when Contains() returns true. Shapes with no radial
        /// structure write 0.
        virtual bool Contains(double _x, double _y, double _z,
                              double* _frac = nullptr) const = 0;
    };

    class BoxShape : public RegionShape {
    public:
        BoxShape(double _x1, double _y1, double _x2, double _y2)
            : m_x1(_x1), m_y1(_y1), m_x2(_x2), m_y2(_y2)
        {
        }

        /// Boxes are altitude-independent: a vertical column spanning all
        /// heights, which is the convention WindRegionBox documents and what
        /// ground-level gust patches rely on. Only cone segments have
        /// vertical extent.
        bool Contains(double _x, double _y, double,
                      double* _frac = nullptr) const override
        {
            if (_frac) { *_frac = 0.0; }
            return _x >= m_x1 && _x <= m_x2 && _y >= m_y1 && _y <= m_y2;
        }

    private:
        double m_x1, m_y1, m_x2, m_y2;
    };

    /**
     * @brief A tapered frustum: point-in-shape is an axis projection (is it
     * within [0, length] downstream of origin?) plus a radial bound that
     * grows linearly from r_start to r_end over that span.
     *
     * The radial bound is measured in the full plane perpendicular to the
     * axis, vertical offset included. The segment is therefore a horizontal
     * tube centred on the rotor hub, extending roughly one rotor radius above
     * and below the hub near the disk and widening downstream.
     *
     * Measuring the radius laterally only, with origin.z unused, made each
     * region unbounded in altitude: a link far above the blade tips or below
     * the turbine tested as inside the wake and received the full
     * rotor-height deficit.
     *
     * The axis remains horizontal (wind direction); the radius carries the
     * vertical extent.
     */
    class ConeSegmentShape : public RegionShape {
    public:
        ConeSegmentShape(
            const gz::math::Vector3d& _origin,
            const gz::math::Vector2d& _axis,
            double _length,
            double _rStart,
            double _rEnd)
            : m_origin(_origin), m_axis(_axis), m_length(_length),
              m_rStart(_rStart), m_rEnd(_rEnd)
        {
        }

        bool Contains(double _x, double _y, double _z,
                      double* _frac = nullptr) const override
        {
            if (_frac) { *_frac = 0.0; }
            double dx = _x - m_origin.X();
            double dy = _y - m_origin.Y();
            double dz = _z - m_origin.Z();
            double d = dx * m_axis.X() + dy * m_axis.Y();
            if (d < 0.0 || d > m_length) {
                return false;
            }
            double lateral = dx * -m_axis.Y() + dy * m_axis.X();
            double r = m_rStart + (m_rEnd - m_rStart) * (d / m_length);
            // Radial distance from the (horizontal) axis line, so the bound
            // applies equally to sideways and vertical offset.
            const double rr = lateral * lateral + dz * dz;
            if (rr > r * r) {
                return false;
            }
            if (_frac && r > 0.0) {
                *_frac = std::sqrt(rr) / r;
            }
            return true;
        }

    private:
        gz::math::Vector3d m_origin;
        gz::math::Vector2d m_axis;
        double m_length, m_rStart, m_rEnd;
    };

    /**
     * @brief Build the RegionShape a WindRegion message describes. The only
     * place in the plugin that knows about `shape_type` — everything else
     * only ever calls RegionShape::Contains().
     */
    /**
     * @brief Speed at radial fraction @p _frac (0 axis, 1 boundary) from a
     * profile sampled evenly across the radius. Linear interpolation.
     */
    static double SampleRadial(const std::vector<double>& _profile, double _frac);

    static std::shared_ptr<const RegionShape> MakeShape(
        const lotusim_msgs::msg::WindRegion& _region);

    struct RegionState {
        std::string id;
        // Shared (not unique) because RegionState is copied wholesale every
        // Update() tick to snapshot m_regions outside the lock (see
        // ResolveWind's doc comment below) — shapes are immutable once
        // built, so sharing them across that copy is safe and avoids
        // reconstructing geometry 500 times a second.
        std::shared_ptr<const RegionShape> shape;
        gz::math::Vector3d velocity = gz::math::Vector3d::Zero;
        double turbulence_sigma = 0.0;
        bool enable_wind = false;
        /// Speed across the radius, axis first, boundary last. Empty means
        /// radially uniform at `velocity` (every pre-existing publisher).
        std::vector<double> radial_speed;

        bool Contains(double _x, double _y, double _z,
                      double* _frac = nullptr) const
        {
            return shape && shape->Contains(_x, _y, _z, _frac);
        }
    };

    /**
     * @brief Resolve the effective wind vector for a world position from a
     * snapshot of the latest ROS state: last matching region in the list
     * wins, otherwise the ambient/global vector. Returns false if the
     * resolved source has wind disabled (no force to apply). Takes the
     * snapshot by parameter (rather than reading members directly) so it can
     * be called per-link, per-tick without re-locking m_wind_mutex each time.
     *
     * `_sigma` receives the turbulence amplitude belonging to whichever
     * source won — the region's own `turbulence_sigma`, or `_ambientSigma`
     * for a link outside every region — so the gust a link feels changes
     * as it flies into and out of a wake, from the same single lookup that
     * already decides its mean wind.
     */
    static bool ResolveWind(
        double _x,
        double _y,
        double _z,
        const std::vector<RegionState>& _regions,
        const gz::math::Vector3d& _globalWind,
        bool _globalEnableWind,
        double _ambientSigma,
        gz::math::Vector3d& _wind,
        double& _sigma);

    /**
     * @brief Advance one link's turbulence state by `_dt` and return the
     * fluctuation velocity [m/s] to add to its mean wind.
     *
     * The state is a three-component, unit-variance Ornstein-Uhlenbeck
     * process — the exact discrete solution of a first-order low-pass filter
     * driven by white noise, which is the Dryden turbulence model's own
     * longitudinal form (MIL-F-8785C) reduced to a single time constant:
     *
     *     n <- exp(-dt/T) * n + sqrt(1 - exp(-2 dt/T)) * N(0,1)
     *
     * The two coefficients are what make it *exactly* stationary at unit
     * variance for any dt, rather than approximately so for small dt: they
     * satisfy a^2 + b^2 = 1, so Var(n) is preserved step to step even when
     * the physics step changes. White noise on its own would be the wrong
     * disturbance to inject here — a multirotor's attitude loop rejects it
     * almost entirely, whereas the 1-3 s gusts this filter produces sit
     * squarely in the band its position loop has to fight.
     *
     * The stored process is dimensionless. Amplitude is applied at use time
     * (sigma * n), so a link crossing a wake edge sees the *amplitude* change
     * with no discontinuity in the underlying process — which is what a real
     * vehicle entering a more turbulent region experiences, as opposed to the
     * jump that scaling the state itself would produce.
     *
     * Components are resolved along the local mean wind: `_sigma` along it,
     * `_sigma * m_sigma_ratio_v` horizontally across it, and
     * `_sigma * m_sigma_ratio_w` vertically (IEC 61400-1 normal turbulence
     * model ratios, 0.8 and 0.5, since atmospheric turbulence is not
     * isotropic).
     */
    gz::math::Vector3d AdvanceGust(
        gz::sim::Entity _entity,
        double _dt,
        double _sigma,
        const gz::math::Vector3d& _wind);

    std::shared_ptr<spdlog::logger> m_logger;
    std::string m_world_name;
    double m_scaling_factor{1.0};

    // Turbulence. All defaults leave the gust term identically zero.
    double m_ambient_sigma{0.0};
    double m_turbulence_time_constant{1.0};
    double m_sigma_ratio_v{0.8};
    double m_sigma_ratio_w{0.5};
    // Explicitly seeded so a run is reproducible: two runs of the same
    // scenario with the same seed see the same gusts, which is what makes a
    // paired comparison between two conditions a comparison of the conditions
    // rather than of two different noise draws.
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normal{0.0, 1.0};
    // Per-link OU state, keyed by entity. Touched only from the sim thread
    // (Update), never from the ROS spin thread, so it needs no lock.
    std::unordered_map<gz::sim::Entity, gz::math::Vector3d> m_gust_state;

    rclcpp::Node::SharedPtr m_ros_node;
    rclcpp::Subscription<lotusim_msgs::msg::Wind>::SharedPtr m_wind_sub;
    rclcpp::Subscription<lotusim_msgs::msg::WindRegionArray>::SharedPtr
        m_regions_sub;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr m_ros_executor;
    std::shared_ptr<std::thread> m_ros_node_thread;

    // Latest wind state received by the spin thread; read (copied) once per
    // Update() on the main sim thread. Mirrors the staging pattern used by
    // PhysicsInterfacePlugin for the same reason: the ROS callbacks run on a
    // dedicated thread, the ECM is only touched from the sim thread.
    mutable std::mutex m_wind_mutex;
    gz::math::Vector3d m_global_wind{gz::math::Vector3d::Zero};
    bool m_global_enable_wind{false};
    std::vector<RegionState> m_regions;

    // Throttles the diagnostic summary logged in Update() (see .cpp) so it
    // fires roughly once a second instead of every physics step.
    uint64_t m_update_count{0};
};

}  // namespace lotusim::gazebo
#endif
