# Waypoint Follower

The `waypoint_follower_plugin` is a Gazebo world plugin that provides lightweight, kinematic 2D waypoint navigation for vessels, an alternative to full physics-driven navigation via the Physics Subsystem. It doesn't model forces or dynamics; it directly scripts position and heading using PID-controlled linear and angular velocity.

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│               WaypointFollowerPlugin : Gazebo World Plugin         │
│            (one instance for the entire simulation world)          │
│                                                                    │
│   Update() ── EachNew<ModelSdf> ──► queue vessel for load()        │
│           └── EachRemoved<ModelSdf> ──► erase all per-entity state │
│                                                                    │
│   Per-entity state (keyed by gz::sim::Entity, all unordered_map):  │
│     m_waypoints / m_waypoints_geo   — local XY & lat/lon targets   │
│     m_waypoint_state                — index of current target      │
│     m_velocities                    — current [linear, angular]    │
│     m_linear_pid / m_angular_pid    — PID gains                    │
│     m_loop, m_rangeTolerance, accel/velocity limits                │
│                                                                    │
│   ROS 2 per vessel: `<name>/waypoints` (service),                  │
│                      `<name>/stop` (service),                      │
│                      `<name>/waypoint_reached` (publisher)         │
└────────────────────────────────────────────────────────────────────┘
```

---

## World SDF Declaration

```xml
<plugin filename="waypoint_plugin" name="lotusim::gazebo::WaypointFollowerPlugin">
</plugin>
```

---

## Enabling a Vessel

A vessel is only picked up by this plugin if its SDF has a `waypoint_follower` element inside `lotus_param`:

```xml
<lotus_param>
  <waypoint_follower>
    <follower>
      ...
    </follower>
  </waypoint_follower>
</lotus_param>
```

If `waypoint_follower` is absent, the plugin ignores the vessel. If `waypoint_follower` is present but has no `<follower>` child, default values are still initialised but there's nothing to navigate to.

---

## Trajectory Modes

Exactly one mode should be set per `<follower>` block: `waypoints`, `circle`, or `line`. They're checked in that priority order (if `<waypoints>` is present, `<circle>` and `<line>` are ignored, even if also present).

### `waypoints`

Explicit list of local `(x, y)` targets:

```xml
<follower>
  <loop>true</loop>
  <waypoints>
    <waypoint>25 0</waypoint>
    <waypoint>15 0</waypoint>
  </waypoints>
</follower>
```

### `circle`

Generates 8 waypoints around the vessel's spawn position, spaced evenly (45° apart) around the given radius:

```xml
<follower>
  <loop>true</loop>
  <circle>
    <radius>20</radius>
  </circle>
</follower>
```

### `line`

Generates exactly 2 waypoints: the vessel's spawn position, and a point `length` meters away in `direction` (radians, world frame):

```xml
<follower>
  <loop>true</loop>
  <line>
    <direction>0</direction>
    <length>5</length>
  </line>
</follower>
```

All generated/declared waypoints are also converted to lat/lon (`m_waypoints_geo`) using the world's spherical coordinate origin, for reporting over ROS.

---

## SDF Parameters

| Parameter | Required? | Default | Description |
|---|---|---|---|
| `<follower>` | — | — | Enables waypoint following for this vessel; must contain one trajectory mode |
| `<loop>` | Optional | `false` | Restart from the first waypoint after reaching the last, instead of stopping |
| `<range_tolerance>` | Optional | `0.5` m | Distance within which a waypoint counts as "reached" |
| `<linear_accel_limit>` | Optional | `0.5` m/s² | Max rate of change of linear velocity. Negative values in SDF are treated as `999.0` (unlimited) |
| `<angular_accel_limit>` | Optional | `0.01` rad/s² | Max rate of change of angular velocity. Negative values treated as `999.0` |
| `<linear_velocities_limits>` | Optional | `(1, 10)` m/s | `(min, max)` linear speed. Negative components treated as `999.0` |
| `<angular_velocities_limits>` | Optional | `0.05` rad/s | **Single value** (max only, no min) |
| `<linear_pid>` | Optional | `(0.5, 0.05, 0.1)` | `(Kp, Ki, Kd)` for linear velocity |
| `<angular_pid>` | Optional | `(0.8, 0.05, 0.4)` | `(Kp, Ki, Kd)` for heading control |

---

## Control Loop

Each `Update()` tick, per vessel:

**1. Target selection** - the current waypoint (`m_waypoint_state[entity]`) is compared against the vessel's pose to compute bearing and distance, both in the vessel's local frame.

**2. Linear velocity (PID on distance error):**

```
desired_velocity = Kp·distance_error + Ki·∫distance_error + Kd·d(distance_error)/dt
desired_velocity *= cos(angle_to_goal)     # slow down while turning; never reverse
desired_velocity = clamp(desired_velocity, linear_velocities_limits)
velocity_change   = clamp(desired_velocity - current_velocity, ±linear_accel_limit·dt)
```

Integral term is anti-windup clamped to ±20% of the max linear velocity limit, divided by `Ki`.

**3. Angular velocity (PID on heading error):**

```
desired_w = Kp_heading·heading_error + Ki_heading·∫heading_error + Kd_heading·(-yaw_rate)
desired_w = clamp(desired_w, ±angular_velocities_limits)
```

This one uses `m_angular_pid` from SDF. Derivative term is computed on measurement (yaw rate) rather than on error, to avoid derivative kick. Integral term is anti-windup clamped, with additional back-calculation when the output saturates, and decays by 5%/tick once heading error drops below ~2.9° (0.05 rad).

**4. Pose update** - velocity is integrated directly into position/yaw (kinematic, not physics-based):

```
x   += v·cos(yaw)·dt
y   += v·sin(yaw)·dt
yaw += ω·dt
```

**5. Goal tracking** - if within `range_tolerance` of the current waypoint: publish status, then either advance to the next waypoint, loop back to the first (if `loop: true`), or stop and remove the vessel from active tracking (if not looping and this was the last waypoint).

---

## ROS 2 Interfaces

Per vessel (namespaced by vessel name):

| Interface | Type | Purpose |
|---|---|---|
| `<name>/waypoints` | Service (`lotusim_msgs/srv/SetWaypoints`) | Override the vessel's current waypoints at runtime with a new `geographic_msgs/GeoPath`. Also resets `loop` and the current waypoint index to 0 |
| `<name>/stop` | Service (`std_srvs/srv/Empty`) | Immediately zero the vessel's velocity and clear its waypoint-follower state |
| `<name>/waypoint_reached` | Publisher (`lotusim_msgs/msg/WaypointFollowerStatus`) | Current waypoint index and remaining path. Published both on reaching a waypoint and periodically per `<update_rate>` |

The plugin runs its own `rclcpp::Node` (named `waypoint_follower`, namespaced to the Gazebo world) on a dedicated thread with a `MultiThreadedExecutor`, independent of other subsystems' ROS nodes.

---

## Lifecycle

- **`EachNew<ModelSdf>`** - detects newly spawned models each tick, checks for `lotus_param > waypoint_follower`, and queues them for `load()` (which does the actual SDF parsing and default initialisation described above)
- **`EachRemoved<ModelSdf>`** - on vessel deletion, erases *all* per-entity state across every internal map, and resets the vessel's ROS publisher so its topic disappears
- **`stopVessel()`** - used both by the `<name>/stop` service and internally when a non-looping vessel finishes its path; zeroes velocity and clears waypoint/PID state (but does **not** erase acceleration/velocity-limit or PID-gain maps, unlike full removal via `EachRemoved`)