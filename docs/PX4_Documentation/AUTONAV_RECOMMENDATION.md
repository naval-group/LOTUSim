# Autonomous Navigation

Use **PX4 OFFBOARD** for realistic drone autonomy, and make the algorithm
replaceable behind a small ROS 2 interface.

Recommended split:

```text
mission/config/sensors
    -> algorithm backend
    -> PX4 OFFBOARD bridge
    -> PX4
    -> Gazebo X500
```

## Why PX4 OFFBOARD

PX4 OFFBOARD lets ROS choose where the drone should go while PX4 still controls
the aircraft.

That means:

- ROS can run custom autonomy algorithms;
- PX4 still handles flight control;
- QGroundControl can still monitor the vehicle;
- pilot/manual takeover remains possible;
- the simulation is closer to real PX4 drone behavior.

This is different from QGC `AUTO.MISSION`, where PX4 owns the whole mission.

## What Needs To Be Built


Create a small ROS 2 package, for example:

```text
src/px4_offboard_autonav/
    px4_offboard_autonav/
        offboard_node.py
        px4_io.py
        frames.py
        mission_loader.py
        backend_loader.py
    launch/
        px4_offboard_autonav.launch.py
```

## Responsibilities

| Part | Responsibility |
| --- | --- |
| `px4_io.py` | Publish/subscribe PX4 messages using `px4_msgs`. |
| `frames.py` | Convert ROS/Gazebo ENU to PX4 NED. |
| `mission_loader.py` | Load waypoints or scenario config. |
| `backend_loader.py` | Select which algorithm backend to use. |
| `offboard_node.py` | Run OFFBOARD heartbeat, state machine, and setpoint loop. |

## Algorithm Backend Idea

Each algorithm should have the same simple interface:

```text
input:
    current vehicle state
    mission/goal
    optional map/sensor data

output:
    next position setpoint
    or next velocity setpoint
```

Example backends:

| Backend | Purpose |
| --- | --- |
| `px4_position` | Simple baseline: send waypoint position setpoints to PX4. |
| `ompl_planner` | Generate collision-free paths. |
| `nav2_style` | Use waypoint/task behavior concepts from Nav2. |
| `ruckig_trajectory` | Smooth trajectory generation. |
| `custom_algorithm` | Project-specific logic. |

## Recommended First Backend

Start with:

```text
px4_position
```

Flow:

```text
current waypoint
    -> position setpoint
    -> PX4 OFFBOARD
    -> PX4 position controller
```

This avoids writing PID/carrot-chase math first. PX4 already has a position
controller, so use it.

## When To Use More Advanced Algorithms

Add a planner backend only when needed:

| Need | Suggested Algorithm / Library |
| --- | --- |
| Basic waypoint following | PX4 position setpoints |
| Obstacle-aware path planning | OMPL |
| Smooth motion constraints | Ruckig or TOPPRA |
| Task/behavior sequencing | Nav2-style behavior tree concepts |
| Fully custom behavior | Custom backend |

## Required PX4 OFFBOARD Pieces

PX4 OFFBOARD requires:

- Micro XRCE-DDS Agent;
- `px4_msgs`;
- `OffboardControlMode` heartbeat;
- `TrajectorySetpoint`;
- PX4 state subscriptions such as `VehicleStatus` and `VehicleLocalPosition`;
- ENU/NED conversion;
- safety behavior when the pilot leaves OFFBOARD.

## Direct ROS / LOTUSim Waypoint Alternative

If PX4 realism is not needed, use the direct LOTUSim waypoint follower.

Flow:

```text
waypoint config
    -> LOTUSim waypoint follower
    -> Gazebo model moves directly
```

This is simpler, but it does not test PX4 behavior, OFFBOARD mode, arming,
failsafes, or real drone control.

## Why The Existing Waypoint Follower Is Not Enough

The existing waypoint follower is useful for simple simulation movement, but it
is not the right place to test PX4-based autonomous navigation.

Main reasons:

- it moves the simulated vehicle directly instead of commanding PX4;
- it does not use PX4 OFFBOARD mode;
- it does not publish `px4_msgs` setpoints;
- it does not test uXRCE-DDS communication;
- it does not test PX4 arming, mode switching, failsafes, or manual override;
- it does not match how a real PX4 drone would receive navigation commands.

So it can be kept as a lightweight simulation-only path, but it should not be
used as the main implementation if the goal is to test algorithms that will
eventually control a PX4 vehicle and want a realistic aeiral navigation.

## Final Recommendation

For autonomous navigation with interchangeable algorithms:

```text
1. Keep PX4 as the flight controller.
2. Add a small ROS 2 PX4 OFFBOARD bridge.
3. Make algorithms plugin-like backends.
4. Start with PX4 position setpoints.
5. Add OMPL/Nav2/Ruckig/custom backends only when needed.
```

This gives flexibility without turning the PX4 bridge into a pile of custom
navigation math.
