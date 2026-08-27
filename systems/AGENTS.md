# AGENTS.md - LOTUSim

Documentation for coding agents and new contributors working in this repository. LOTUSim is an open-source (EPL-2.0) multi-agent naval simulator: ROS2 + Gazebo for orchestration and rendering, xdyn for rigid-body dynamics, coupled in co-simulation.

## Architecture overview

LOTUSim's Core Layer is built on Gazebo and organizes functionality into 6 **subsystems**: Physics, Sensor, Power, Rendering, Multi-agent, and Environment. Each are implemented as an independent Gazebo plugin that can be enabled, disabled, or replaced without affecting the others.

Three more layers sit around that core:

- **Internal Interface Layer** : the communication bridge (abstract base classes) between a subsystem and an external server.
- **Internal Computation Layer** : user-operated **Engines** (Physics, Sensor, Power). Each Engine is a standalone server that receives state from its Interface, runs the relevant models, and returns results. This layer is external to the core of LOTUSim and can scale on multiple agents.
- **External Interface Layer** : how user external applications (e.g.: navigation stacks, fleet managers, ...) talk to the simulation, primarily via a ROS 2 / FastDDS interface, plus a lightweight WebSocket interface.

This client-server split is the reason LOTUSim can distribute computational load across machines instead of bottlenecking on one.

A few terms worth noting:

- **Entity** : anything in the simulation world (vessel, sensor, mesh).
- **Agent** : an entity controlled by an external system via the ROS 2 interface (e.g. a vessel driven by xdyn or a Nav2 stack). Not every entity is an agent (a static obstacle is an entity but not an agent).
- **`lotus_param`** : the custom XML block inside a model's SDF that each Subsystem plugin reads on spawn to configure itself. A model without this block is invisible to LOTUSim's plugins.

Full details, diagrams, and the plugin lifecycle (PreUpdate → Update → PostUpdate) are on the wiki's [Architecture](https://github.com/naval-group/LOTUSim/wiki/Architecture) page.

## How the components fit together

LOTUSim runs as a co-simulation between three components:

- **Gazebo (`gz`)** : world owner and, optionally, rendering.
- **xdyn (optional)** : rigid-body dynamics, run as its own external process, not a Gazebo plugin. Gazebo's `physics_engine_interface` is a websocket *client* that connects to an `xdyn-for-cs` server (one per vessel, each on its own TCP port).
- **Unity (optional)** : an alternative renderer to the Gazebo GUI, used by models that ship without a `<visual>` (see below).

`lotusim run` starts `gz` only. To simulate a vessel with dynamics, start its `xdyn-for-cs` server first, e.g.:

```bash
lotusim run
xdyn-for-cs $HOME/lotusim_ws/src/LOTUSim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
```

Without an `xdyn-for-cs` listening on the matching port, the vessel spawns and renders but has no dynamics (`XdynWebsocket::onFail`).

## Rendering: Gazebo GUI vs. Unity

Some models (e.g. `wamv`, `dtmb_hull`, `lrauv`) ship collision geometry only and render through Unity via `render_plugin` → ROS2 → Unity client; so they won't appear in the Gazebo GUI by default. To see a vessel directly in `gz` without Unity, toggle *Entity Tree → right-click → View → Collisions*, or add a `<visual>` to the model.

## Environment

- Ubuntu 24.04 → ROS2 Jazzy + Gazebo Harmonic
- ROS setup scripts are bash-only. Under zsh, source the `.zsh` variant (`setup.zsh`) instead, or use the `lotusim` wrapper (shebang `#!/bin/bash`).

Two ways to install:

### Container (recommended)

A prebuilt image ships the full ROS2 + Gazebo + built LOTUSim workspace:

```bash
docker pull ghcr.io/naval-group/lotusim:latest
docker run --rm -it ghcr.io/naval-group/lotusim:latest
```

To build/test a local change against the image's prebuilt workspace, mount your source, rebuild only the affected package:

```bash
docker run --rm -v "$PWD":/lotusim_ws/src/LOTUSim ghcr.io/naval-group/lotusim:latest \
  bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source /lotusim_ws/install/setup.bash \
           && cd /lotusim_ws && colcon build --merge-install --packages-select <pkg>'
```

Headless (no GUI): good for builds, tests, and headless runs.

### Building from source

```bash
lotusim install        # one-time: deps + build
lotusim build          # rebuild workspace (colcon, --merge-install); clean_build to start over
colcon build --merge-install --packages-select <pkg>   # single package
```

## Running a world

- **With physics:** start one `xdyn-for-cs` per vessel (each on its own port), then `lotusim run <world>.world`.
- **Without physics** (render / plugin debug): `lotusim run --gui <world>.world`.

## Launching an example scenario

There are 3 ways to launch a scenario:

1. **Web UI** (best for quick, one-off scenarios with up to ~5 models):
   `lotusim run` + `lotusim ui`, then use the **Scenarios** tab to add vessels, choose a model, and enable the plugins you want (Rendering, Physics via xdyn, and/or Waypoint Follower). Start one `xdyn-for-cs` per vessel using the physics engine, matching the port you set in the UI, then launch the scenario from the **Home** page.

2. **Script-based** (C++, Python, or Jupyter: see the [`examples`](https://github.com/naval-group/LOTUSim/tree/main/examples) folder):
   a script spawns vessels via the `mas_cmd` action, sends an SDF `lotus_param` snippet to configure their interfaces, and sends thruster
   commands over `vessel_cmd_array`. You can check out the `LOTUSim/examples` folder.

3. **`LOTUSim-generic-scenario`** package (best for spawning many agents at once from a reusable JSON config):
   runs through its own standalone Unity executable rather than `lotusim ui` / `lotusim run`. Agents are defined in a JSON file specifying model, count, and initial poses (either `[lat, lon(, alt)]` or a full 6-element `[x, y, z, roll, pitch, yaw]`). See the package's own README for the full build/launch sequence.

## Controlling a vessel

There are two ways depending on what you need:

Vessels are spawned via the Multi-Agent System's ROS 2 action interfaces (`mas_cmd` for one vessel, `mas_cmd_array` for several). `CREATE_CMD` sends a full `lotus_param` block as a string (`sdf_string`). What you put in that block decides which of the two control paths below you get.

### Waypoint follower (kinematic, no physics engine needed)

Good for scripted, predictable movement without an `xdyn-for-cs` server running. Two ways to drive it:

- **Preset pattern at spawn** : declare a `<waypoint_follower>` in the `lotus_param` sent with `CREATE_CMD`, e.g. a looping circle:
```xml
  <lotus_param>
      <waypoint_follower>
          <follower>
              <loop>true</loop>
              <linear_accel_limit>0.5</linear_accel_limit>
              <angular_accel_limit>0.005</angular_accel_limit>
              <angular_velocities_limits>0.01</angular_velocities_limits>
              <range_tolerance>2</range_tolerance>
              <circle>
                  <radius>10</radius>
              </circle>
          </follower>
      </waypoint_follower>
  </lotus_param>
```
- **Custom path at runtime** : call the vessel's `<vessel_name>/waypoints` service (`lotusim_msgs/srv/SetWaypoints`) with a list of `GeoPoint`s:
```python
  request = SetWaypoints.Request()
  request.path = [GeoPoint(latitude=lat, longitude=lon, altitude=alt)]
  request.loop = False
  future = self.waypoint_client[model_name].call_async(request)
```

### Full dynamics (via xdyn)

Needed for physically accurate motion driven by thruster commands. Requires `xdyn-for-cs` running for that vessel (see [Running a world](#running-a-world)).

1. **Declare the physics interface at spawn**, matching the domain the vessel starts in (`underwater`, `surface`, or `aerial`), the xdyn
   server's URI, and a name for each thruster:
```xml
   <lotus_param>
       <physics_engine_interface>
           <underwater>
               <interface_type>XDynWebSocket</interface_type>
               <uri>ws://127.0.0.1:12346</uri>
               <thrusters>
                   <thrusters1>propeller</thrusters1>
               </thrusters>
           </underwater>
           <init_state>Underwater</init_state>
       </physics_engine_interface>
   </lotus_param>
```
2. **Publish thruster commands** on `/lotusim/vessel_cmd_array` (`lotusim_msgs/msg/VesselCmdArray`). Each command's `cmd_string` is JSON
   keyed by the thruster name declared above:
```python
   cmd = VesselCmd()
   cmd.vessel_name = vessel_name
   cmd.cmd_string = json.dumps({"propeller(rpm)": 200.0, "propeller(P/D)": 0.88})
   cmd_array.cmds.append(cmd)
   self.cmd_publisher.publish(cmd_array)
```

> ⚠️ The thruster name declared in the SDF (`<thrusters1>propeller</thrusters1>`) and the key in the command JSON (`"propeller(rpm)"`) 
> must match exactly. A mismatch will fail and the vessel just won't move.

### Tracking and cleanup

- Vessel positions stream on `/lotusim/poses` (`lotusim_msgs/msg/VesselPositionArray`) subscribe to track spawned vessels.
- On shutdown, delete everything you spawned via `DELETE_CMD` on `mas_cmd_array`, referencing each vessel by the name given at spawn. 
  Handle this in a signal handler block so Ctrl+C doesn't leave vessels behind.

Full runnable examples: [`python-scripts/`](https://github.com/naval-group/LOTUSim/tree/main/examples) in the `examples` folder.


### Adding a vehicle

A model lives in `assets/models/<name>/`:

- `model.config`
- `model.sdf` : SDF format; version varies by model today (1.6–1.10). This is also where you declare a model's sensors and battery/ generator, and where the `lotus_param` block goes. A model without `lotus_param` won't be picked up by LOTUSim's plugins.
- `.stl` mesh for the model
- an xdyn `.yml` dynamics file : naming varies by model (`wamv.yaml`, `dtmb-xdyn.yml`, `fremmConfig.yaml`, …), declaring:
  - `external forces` : passive forces the model always experiences, e.g. gravity, quadratic damping.
  - `controlled forces` : propulsion you command at runtime, e.g. a `wageningen B-series` thruster, or `propeller+rudder` for rudder-steered
    vessels. Give each a unique `name`. That name is what you address it by when sending thruster commands.
  - See [Forces & Propulsion Types](https://github.com/naval-group/LOTUSim/wiki/forces-&-propulsion-(xdyn)) for the full list. Re-launch your scenario after editing the `.yml` for changes to take effect.

Wire the model into a world via `<include>` + `<lotus_param>`. See [Extend with your components](https://github.com/naval-group/LOTUSim/wiki/extend-with-your-components) for adding new sensor/power-provider types rather than just using existing ones.


## Coordinate conventions

xdyn is NED, right-handed; Gazebo is ENU. Quaternion wire order is `qr, qi, qj, qk` (= w, x, y, z), by name. Gazebo↔Unity applies a `Z → -Y` transform on the Unity side.

## Couple of things to note

- **Process cleanup:** `pkill -f "gz sim"` or `pkill -f xdyn-for-cs` will match your own shell's command line too, so it can kill itself along with the target process. Capture the PID (`pid=$!`) and `kill "$pid"`, or use a `trap cleanup EXIT`.
- **Restarting a run:** if a previous `gz`/`xdyn-for-cs` session didn't shut down cleanly, check for lingering processes on the ports you plan to use before starting a new one.

## Contributing

Issue (with the appropriate label) → announce yourself → fork → implement → test → PR referencing the issue (see `CONTRIBUTING.md`). EPL-2.0: never vendor GPL or non-redistributable assets. Whoever opens the PR answers for every line, AI-assisted or not.

## Related repositories (naval-group)

LOTUSim spans several repos, this core is only the simulation backend.

- **LOTUSim** this repo: the simulator core (ROS2 + Gazebo + xdyn co-sim)
- **LOTUSim-Xdyn** : the xdyn physics engine, a fork/mirror of the upstream at `gitlab.com/sirehna_naval_group/sirehna/xdyn`. 
- **LOTUSim-generic-scenario** : reference controllers/scenarios (`src/agents/`), the auto-installer, and a prebuilt Unity player.
- **LOTUSim-Unity-modules** : frontend (an alternative to the Gazebo GUI).
- **LOTUSim-UI-frontend** (React), **LOTUSim-UI-backend** the web UI.

## Further reading

- **Project wiki** : full documentation (installation, models, architecture, tutorials, FAQ): https://github.com/naval-group/LOTUSim/wiki
- **Getting Started** : fastest path from zero to a running simulation https://github.com/naval-group/LOTUSim/wiki/Getting-Started
- **Architecture** : subsystems, interfaces, engines, and the `lotus_param` block in full detail: https://github.com/naval-group/LOTUSim/wiki/Architecture
- **Tutorial** : creating scenarios, commanding vessels, extending with your components, and customising models step by step: https://github.com/naval-group/LOTUSim/wiki/Tutorial
