---
name: lotusim-developer
description: Build, run, and contribute to LOTUSim (Naval Group's ROS2 + Gazebo + xdyn maritime simulator). Use to install/build LOTUSim, add a vehicle/model, run a simulation with physics, debug a vessel that won't move, or prepare a PR. Essential before any `lotusim`/`gz` command — the architecture (physics = external xdyn co-sim, rendering in Gazebo OR Unity) traps anyone who doesn't know it.
---

# LOTUSim developer

LOTUSim is Naval Group's open-source (EPL-2.0) multi-agent maritime simulator.
Stack: **ROS2 + Gazebo** (orchestration/rendering) + **xdyn** (rigid-body
dynamics, in co-simulation). Repo: `github.com/naval-group/LOTUSim`.

This skill condenses non-obvious, field-verified knowledge. **Read the 6
pitfalls below first** — each one costs hours if ignored.

## The 6 pitfalls (know these BEFORE coding)

1. **Physics is an EXTERNAL xdyn server, not a Gazebo plugin.** Gazebo's
   `physics_engine_interface` is a *websocket client* that connects to an
   `xdyn-for-cs` launched separately (one per vessel, on a TCP port: 12345,
   12346…). **`lotusim run` launches only `gz` — NOT xdyn.** With no xdyn server
   listening, a vessel spawns and renders but has **no dynamics**:
   `XdynWebsocket::onFail` → `loadVessel: Loading failed, Removing physics`.
   → To run with physics: see `references/run-and-verify.md`.

2. **Many models have NO `<visual>`** (collision only). Ships such as `wamv`,
   `dtmb_hull`, `lrauv` render in **Unity** (`render_plugin` → ROS2 → Unity
   client), so in the **Gazebo GUI they are invisible** (toggle *Entity Tree →
   right-click → View → Collisions* to see them). To display a vessel directly
   in gz, give it a `<visual>` — a legitimate LOTUSim pattern (cf. `fremm`,
   `commando`).

3. **Ubuntu 24.04 (noble) → ROS2 Jazzy + Gazebo Harmonic.** Not Humble (= 22.04).
   Harmonic ⇒ prefix **`gz`** (`gz sim`, `gz.msgs.*`), never `ign`/Fortress. The
   `launch/lotusim` wrapper auto-detects the codename and sets `ROS_DISTRO`/
   `GAZEBO_VERSION` on its own. Install: `lotusim install`.

4. **ROS setup scripts are bash-only.** `source /opt/ros/jazzy/setup.bash` breaks
   under zsh (`complete: command not found`, empty `${BASH_SOURCE}` → wrong
   paths, and `CMAKE_PREFIX_PATH`/`AMENT_PREFIX_PATH` unset → a build that can't
   find `geometry_msgs`). Under zsh: source `setup.zsh`, or go through the
   `lotusim` wrapper (shebang `#!/bin/bash`).

5. **`pkill -f "gz sim"` / `pkill xdyn-for-cs` KILL THEMSELVES.** The pattern
   matches your own shell's command line (which contains that text) → self-kill
   (exit ~144). Capture the PIDs (`pid=$!`) and `kill "$pid"`, or use a
   `trap cleanup EXIT`.

6. **In co-sim, THRUST comes neither from the yaml `commands:` block nor from the
   `waypoint_follower`.** `xdyn-for-cs` **ignores** the yaml `commands:` block.
   Thruster setpoints are **published** on the ROS2 topic
   `/<world>/vessel_cmd_array` (`lotusim_msgs/msg/VesselCmdArray`): each
   `VesselCmd.cmd_string` is a JSON `{"<thruster>(rpm)": <val>,
   "<thruster>(P/D)": <val>}` that `physics_interface_plugin` forwards verbatim
   to xdyn. With no publisher → hardcoded default `<thruster>(rpm)=2.0` →
   near-zero thrust (warning `Wageningen … n too small`). The `waypoint_follower`
   emits status only: it is a separate **kinematic** mode, and **no world
   combines it** with `physics_engine_interface`. → Moving a vessel UNDER PHYSICS
   = a small rclpy node publishing on `vessel_cmd_array` (cf.
   `references/run-and-verify.md`). The **reference pattern** lives in
   **`LOTUSim-generic-scenario`** (`lrauv_propeller.py`), **not in the core** —
   that's where controllers live (cf. `references/architecture.md`).

## Quick start

```bash
# Container (reproducible; see AGENTS.md for the mount-and-build pattern)
docker run --rm -it ghcr.io/naval-group/lotusim:latest bash

# Native install / build (bash only)
lotusim install            # ROS2 Jazzy + Gazebo Harmonic + colcon build
lotusim build              # rebuild only; clean_build to start fresh

# Run a world WITH physics (xdyn server(s) + gz) — pitfall #1:
#   start one xdyn-for-cs per vessel (each on its TCP port), then:
lotusim run <world>.world           # add --gui for the Gazebo GUI

# Run a world WITHOUT physics (gz only — render / plugin debug)
lotusim run --gui <world>.world
```

Physics success = log `PhysicsInterfacePlugin::loadVessel: ... Surface init
completed` + `XdynWebsocket::onOpen`. Failure = `onFail` / `unable to connect` →
no xdyn server on the port, or a port/uri that does not match the world.

## Adding a vehicle (the typical contribution)

A model lives in `assets/models/<name>/`: `model.config`, `model.sdf` (SDF 1.10,
minimal: collision + sensors; add a `<visual>` to see it in gz), `<name>.yaml`
(the **xdyn model**: inertia, added mass, damping, propulsion — this is where the
dynamics are), `meshes/` (.dae visual + one .stl for xdyn hydrostatics). The
vehicle is wired into a world via `<include>` + `<lotus_param>`
(`physics_engine_interface` → xdyn port + thrusters; `waypoint_follower`;
`render_interface`). Full templates + protocol: `references/models-and-worlds.md`.

⚠️ **Split:** the **model** (assets) goes in the **core**; its
**controller/scenario** goes in **`LOTUSim-generic-scenario`** (`src/agents/`),
**not** the core (cf. `references/architecture.md`).

PR workflow (`CONTRIBUTING.md`): issue (label `new_model`) → announce yourself →
fork → implement → test → PR referencing the issue. **License EPL-2.0**: never
vendor GPL assets (e.g. ArduPilot SITL_Models) or assets without redistribution
rights (e.g. manufacturer CAD); re-author from public dimensions only.

## References (load as needed)

- `references/architecture.md` — where to find what: the wiki index, the
  **`LOTUSim-generic-scenario`** repo (controllers/scenarios), the Unity repos,
  the ecosystem map; plus the gz↔xdyn↔Unity data flow and coordinate conventions
  (NED/ENU/Unity, quaternion `qr,qi,qj,qk` order, `Z→-Y`).
- `references/models-and-worlds.md` — anatomy of `model.config`/`model.sdf`/
  `<name>.yaml`, the `<lotus_param>` block, visual-vs-Unity, copy-paste
  templates, mesh generation.
- `references/run-and-verify.md` — xdyn co-sim orchestration, `xdyn-for-cs`
  (args, ports), headless verification, the "did the vessel actually move?" oracle.
