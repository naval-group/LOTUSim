# Autonavigation Comparison: PX4 Path vs Direct ROS Waypoint Path

This document compares two ways to implement autonomous navigation for an aerial
agent in LOTUSim:

1. **PX4-based navigation**, where PX4 controls the X500 and ROS sends missions
  or OFFBOARD setpoints to PX4.
2. **Direct ROS / LOTUSim waypoint navigation**, where the original LOTUSim
  waypoint follower moves the simulated model directly through ROS/Gazebo
   integration.

The moints.ain decision is whether the simulated drone should behave like a
PX4-controlled aircraft, or like a generic LOTUSim simulated agent following
wayp

## High-Level Difference


| Question                                  | PX4-Based Navigation                      | Direct ROS / LOTUSim Waypoint Navigation  |
| ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| Who controls the vehicle motion?          | PX4                                       | LOTUSim waypoint follower / Gazebo plugin |
| Is PX4 SITL required?                     | Yes                                       | No                                        |
| Is QGroundControl useful?                 | Yes                                       | Not required                              |
| Uses PX4 flight modes?                    | Yes: OFFBOARD, AUTO.MISSION, POSCTL, etc. | No                                        |
| Uses PX4 failsafes and arming logic?      | Yes                                       | No                                        |
| Talks through uXRCE-DDS / `px4_msgs`?     | Yes for ROS OFFBOARD                      | No                                        |
| Good for testing real drone behavior?     | Yes                                       | Limited                                   |
| Good for simple simulation-only movement? | More setup than needed                    | Yes                                       |


## Option 1: Autonavigation Through PX4

In the PX4 approach, LOTUSim starts an X500 simulation connected to PX4 SITL.
PX4 owns the real flight-control behavior: arming, modes, position tracking,
failsafes, and mission execution.

There are two PX4-style sub-options.

### PX4 AUTO.MISSION

This is the QGroundControl mission path:

```text
QGroundControl
    -> MAVLink
    -> PX4 AUTO.MISSION
    -> PX4 controls the X500 in simulation
```

In this mode, QGC uploads the mission to PX4. PX4 then flies the mission itself.
ROS navigation code is not required for the mission logic.

Use this when:

- the mission can be expressed as normal PX4/QGC waypoints;
- you want PX4-native behavior;
- you do not need custom ROS-side autonomy;
- you want the workflow to match a real PX4 drone.

### ROS OFFBOARD Through PX4

This is the ROS autonomy path:

```text
mission_json / ROS planner / algorithm backend
    -> aerial_navigation
    -> px4_msgs over uXRCE-DDS
    -> PX4 OFFBOARD
    -> PX4 controls the X500 in simulation
```

In this mode, ROS still decides the mission target, but PX4 remains the flight
controller. The ROS node publishes OFFBOARD heartbeat and trajectory setpoints.

The implementation that would be a better choice is:

That means ROS sends position setpoints, and PX4's own position controller
tracks them. This avoids maintaining custom PID/carrot-chase control math in
the scenario repo.

Use this when:

- the project needs ROS-side autonomy;
- missions should be triggered from command/config;
- different planner or controller algorithms need to be tested;
- PX4 behavior still matters;
- QGC/manual takeover should coexist with autonomy.

## Option 2: Direct ROS / LOTUSim Waypoint Navigation

The original LOTUSim waypoint follower is a simulation-side implementation. It
does not command PX4. Instead, it moves a model directly inside the Gazebo /
LOTUSim simulation loop.

The rough flow is:

```text
ROS waypoint service / LOTUSim waypoint config
    -> LOTUSim waypoint follower plugin
    -> Gazebo model pose update
    -> simulated agent moves
```

This is simpler because there is no PX4 SITL, no QGC, no uXRCE-DDS bridge, and
no PX4 message layer.

Use this when:

- the vehicle only needs to move in simulation;
- PX4 flight-controller behavior is not important;
- the agent should behave like a generic LOTUSim model;
- fast scenario testing is more important than PX4 realism.

The limitation is that this path does not test real PX4 behavior. It bypasses
PX4 flight modes, arming, failsafes, and position-control behavior. For an
aerial drone, it may also need extension if the existing waypoint follower is
primarily 2D and the X500 needs full 3D waypoint behavior with altitude and yaw.

## Architecture Comparison

### PX4-Based ROS OFFBOARD

```text
Mission command/config
        |
        v
aerial_navigation/nav_node.py
        |
        v
algorithm backend
  - px4_position
  - external
  - legacy_pid
  - legacy_carrot_chase
        |
        v
px4_io.py
        |
        v
px4_msgs / uXRCE-DDS
        |
        v
PX4 SITL
        |
        v
Gazebo X500
```

### Direct ROS / LOTUSim Waypoint

```text
Waypoint command/config
        |
        v
LOTUSim waypoint follower plugin
        |
        v
Gazebo / LOTUSim model update
        |
        v
Simulated agent moves
```

## Strengths and Weaknesses

### PX4-Based Navigation Strengths

- Closer to real drone deployment.
- Tests PX4 flight modes and controller behavior.
- Supports QGC monitoring and manual override.
- Allows ROS autonomy while still using PX4 as the flight controller.
- Better fit if the X500 is meant to represent a PX4-controlled UAV.

### PX4-Based Navigation Weaknesses

- More setup: PX4 SITL, QGC, uXRCE-DDS for ROS OFFBOARD.
- More moving parts to debug.
- Requires correct ENU/NED conversion.
- ROS OFFBOARD requires continuous heartbeat and setpoint streaming.

### Direct ROS / LOTUSim Waypoint Strengths

- Simpler to run.
- No PX4 dependency.
- Good for quick simulation-only waypoint movement.
- Reuses existing LOTUSim Core functionality.

### Direct ROS / LOTUSim Waypoint Weaknesses

- Does not validate PX4 behavior.
- Does not use QGC missions, arming, failsafes, or PX4 modes.
- May not represent realistic aerial flight control.
- Existing implementation may need extension for full 3D aerial navigation.

## Recommended Use

Use **PX4-based navigation** when the goal is to simulate a PX4-controlled X500
or prepare behavior that could later map to a real PX4 drone.

Use **direct ROS / LOTUSim waypoint navigation** when the goal is only to move an
agent through the LOTUSim world for scenario testing and PX4 realism is not
needed.

For the current PX4 path, the recommended baseline is:

```bash
ros2 launch aerial_navigation aerial_nav.launch.py \
  algorithm:=px4_position \
  mission_json:='[[30,0,20,0],[30,30,20,1.57]]'
```

This keeps mission control in ROS, but lets PX4 handle the actual position
tracking. If a new planning library is needed later, it can be plugged in with:

```bash
ros2 launch aerial_navigation aerial_nav.launch.py \
  algorithm:=external \
  algorithm_plugin:=my_pkg.my_backend:create_backend
```

That gives the project a clean test point for existing navigation libraries
without rewriting the PX4 bridge each time.