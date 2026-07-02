# Running with physics & verifying

## The co-simulation architecture (the key point)

```
   xdyn-for-cs (websocket server, 1 per vessel)          gz sim (Gazebo)
   ws://127.0.0.1:12345  <───────────────────────────  physics_interface_plugin (client)
        ^                                                       │
        │ dynamics (xdyn reads <name>.yaml)                     │ pose, waypoint, sensors
```

- `physics_interface_plugin` (inside gz) is a **client**; it reads the `<uri>`
  from the world's `<lotus_param><physics_engine_interface>` and connects.
- `xdyn-for-cs` (prebuilt binary in `physics/`) is the **server**: it computes
  the 6-DOF dynamics from `<name>.yaml`. **One process per vessel**, each on a
  unique port (12345, 12346, …).
- `lotusim run` **launches gz ONLY**. You must start xdyn separately, otherwise:
  `XdynWebsocket::onFail: Failed ws://… → loadVessel: Loading failed, Removing physics`.

### Launching xdyn-for-cs

```bash
xdyn-for-cs <model>.yaml --address 127.0.0.1 --port 12345 --dt 0.2
#   --dt 0.2 = time step ; -s rk4 by default (euler/rkck available)
#   --verbose to see the exchanges ; -w for the websocket detail
```

Reference orchestrator (multi-agent, the "real" launcher):
`LOTUSim-generic-scenario/src/simulation_run/executable/scenario_launch.sh`
(starts one `xdyn-for-cs agent.yml --port $port --dt 0.2` per agent, then gz,
and `pkill -f xdyn-for-cs` on cleanup).

For a local run, a small wrapper typically starts one xdyn server per port, then
`gz`, and **kills the servers by PID via a `trap`** (never `pkill` — see runtime
pitfalls).

## Launching gz (Harmonic)

```bash
lotusim run <world>.world          # headless (gz sim -s) — default
lotusim run --gui <world>.world    # with GUI
# direct: gz sim -s -v3 -r <world>   (-s server only, -r run, -v3/-v4 verbosity)
```

The wrapper sets `GZ_SIM_SYSTEM_PLUGIN_PATH=install/lib`,
`GZ_SIM_RESOURCE_PATH=assets/models`. Prefix **`gz`** (Harmonic), not `ign`.

## Verifying (oracle = displacement)

For "it floats + it moves", the robust oracle is the **displacement measured in
the sim** (not the rendering — EGL/OGRE2 is fragile in headless/remote setups;
never gate pass/fail on it). Headless loop:

```
build → validate SDF (gz sdf -k <file>) → [xdyn server(s)] → spawn headless (gz sim -s -r)
      → read start/end pose → assert dist > threshold → (artifact: trajectory plot)
```

Log signals to assert:

- `physics in domain Surface init completed` + `XdynWebsocket::onOpen` → physics OK;
- `onFail` / `unable to connect` → no xdyn server / port-uri mismatch;
- `Failed to load system plugin` → plugin path (`GZ_SIM_SYSTEM_PLUGIN_PATH`) or build;
- `dist ≈ 0` → sank (bad hydrostatics) or no thrust (no setpoint on `vessel_cmd_array`).

### Thrust source in co-sim (PITFALL #6): `vessel_cmd_array`, not the yaml or the waypoint

`xdyn-for-cs` **ignores** the yaml `commands:` block. Thrust is **published** on
the ROS2 topic `/<world>/vessel_cmd_array` (`lotusim_msgs/msg/VesselCmdArray`):
`physics_interface_plugin` subscribes and forwards `VesselCmd.cmd_string` (JSON)
to xdyn. `cmd_string` keys = `"<thruster>(<param>)"`, params `rpm` / `P/D` /
`beta`; the names must match the world's `<thrusters>` AND the yaml's actuators.
With no publisher → hardcoded default `rpm=2.0` → near-zero thrust. The
`waypoint_follower` is NOT this source (status only, kinematic mode). Minimal
publisher:

```python
# pub_thrust.py — makes a vessel move under physics (co-sim)
import json, rclpy
from rclpy.node import Node
from lotusim_msgs.msg import VesselCmd, VesselCmdArray
rclpy.init(); n = Node("thrust_pub")
pub = n.create_publisher(VesselCmdArray, "/lotusim/vessel_cmd_array", 10)
cmd = json.dumps({"PSthruster(rpm)": 250.0, "SBthruster(rpm)": 250.0})
n.create_timer(0.1, lambda: pub.publish(
    VesselCmdArray(cmds=[VesselCmd(vessel_name="blueboat", cmd_string=cmd)])))
rclpy.spin(n)
```

### Reading the pose (oracle) → you need the `SceneBroadcaster`

Many LOTUSim worlds (e.g. `xdyn_multithread_test`) render in Unity and **don't
embed** `gz-sim-scene-broadcaster-system` → no pose topic (and invisible in the
gz GUI). To measure displacement (and
render in gz), add to the world
`<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>`,
then read `/world/<world>/dynamic_pose/info` (`gz topic -e … --json-output`),
extract the vessel pose (jq) and assert the horizontal displacement.

## Runtime pitfalls

- **Shell**: launch via the `lotusim` wrapper (bash) or source `setup.zsh` under zsh.
- **Suicidal pkill**: `pkill -f "gz sim"`/`xdyn-for-cs` matches your own command
  line → self-kill (exit ~144). Capture `$!` and `kill`, or use a `trap`.
- **Ports**: one per vessel; must match the world's `<uri>`.
- **Gravity 0** in the worlds: normal, xdyn does all the dynamics.
- **`set -u`**: NEVER enable it before sourcing the ROS env (`setup.bash`
  references undefined vars → the script dies silently, exit 1 with no output).
- **`gz sdf -k`**: validates a `model.sdf` but **fails on a world** with
  `<include>` (`Unable to find uri[model://…]`, no find-callback) → validate a
  world by spawning it, not with `-k`.
- **Crabbing mesh**: LOTUSim convention = mesh bow on **+y** (cf. `models-and-worlds.md`).
- **GUI screenshot**: the `/gui/screenshot` service wants a **directory** (it
  drops a timestamped PNG there), not a file path.
