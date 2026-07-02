# LOTUSim — ecosystem & data flow (where to find what)

The `naval-group/LOTUSim` core is not self-sufficient: the knowledge and the
runnable pieces are spread across the **wiki** + a **scenario** repo + three
**Unity** repos. This page is the index, plus the gz ↔ xdyn ↔ Unity data flow
and the coordinate conventions.

## The wiki (`github.com/naval-group/LOTUSim/wiki`)

Git-cloneable: `git clone https://github.com/naval-group/LOTUSim.wiki.git`.
The most useful pages:

| Page | Content |
|---|---|
| **Xdyn-User-Guide** (the bible) | NED + body frames (X-fwd, Y-stbd, Z-down), rotation convention `Z/Y/X` `[psi, theta', phi'']` (the only one supported), quaternions `qr,qi,qj,qk`, mesh (normals facing the fluid), control surfaces (`hydrodynamic polar`), propellers (Kt/Kq, prop+rudder, Wageningen B), a full yaml example, units (trap: `ton` = 907 kg). |
| **Core-Development** | the `render_plugin` (SDF), the `<lotus_param>` block (`render_interface`/`physics_engine_interface`), and the gz↔Unity `Z→-Y` frame conversion. |
| **Physics-Implementation** | xdyn co-sim, Unity↔xdyn wave sync (spatially periodic waves, period = the Unity HDRP tile size). |
| **Platform_Model_Characteristics** | per-model sheets (dimensions, masses, frame). |
| **Gazebo-Setup** | `model.config`/`model.sdf`, mesh formats (STL/DAE/OBJ), `<visual>` optional because Unity renders (→ models are invisible in the gz GUI). |
| **Getting-Started** | overview (3 steps: sdf → physics engine → Unity), light; the real walkthrough is in `LOTUSim-generic-scenario` below. |
| Unity-Setup / Unity-Development | the `lotusim_interface` scripts (`common.cs` = pose conversion, `LotusimConnector.cs`). |
| Xdyn-Setup / Xdyn-Development | building xdyn, modules, PR workflow (short feature branch tied to an issue, no direct commit to master, CI mandatory). |

## `naval-group/LOTUSim-generic-scenario` — the runnable walkthrough

The ready-to-use **scenario workspace** on top of the core (covers what the
wiki's Getting-Started only skims). It contains:

- an **automated installer** `install_core_and_generic_scenario.sh`: detects
  Ubuntu→ROS (22.04→Humble, 24.04→Jazzy), clones the core into a colcon
  workspace, configures `.bashrc`, runs `lotusim install`, builds (idempotent).
- a **config-driven runner** `src/simulation_run/executable/scenario_launch.sh
  --config <x>.json` (e.g. `defenseScenario.json`); spawns agents (initial-pose
  formats documented); **thruster control via a ROS topic** (`vessel_cmd_array`, cf. pitfall #6).
- **architecture docs** `doc/DIAGRAMS.md`: package tree, class/sequence diagrams,
  the `defenseScenario` launch sequence, nodes/topics/actions.
- a **prebuilt Linux Unity player** under `lotusim_unity_executables/`.

This is **THE template for an out-of-tree user project** (a separate colcon
workspace that consumes the core). The split it enforces:

- **core** = engine + **model library** (`assets/models/`) + worlds. **No
  controllers**: `vessel_cmd_array` is only *consumed* there by `physics_interface_plugin`.
- **generic-scenario** = **behaviors** (one **ROS2 package per agent**, with the
  usual double package dir: `src/agents/<name>/<name>/<name>.py`) + **scenarios** (JSON config in
  `src/simulation_run/config/`) + bridges (`src/gz_ros2_bridge`) + the Unity player.
- **your own project** = the demo plumbing (scripts, Blender mesh sources, captures).

**Controller pattern** (where a pilot goes):
`src/external_packages/lrauv_propeller/lrauv_propeller/lrauv_propeller.py`
inherits the agent class, creates a publisher on `/<world>/vessel_cmd_array` and sends
`{"<thruster>(rpm)": …}`. That one is **open-loop** (an rpm sequence on a timer)
on a **thruster**; a **closed-loop** controller (P-control on the pose) and/or
**angle-commanded actuators** (sail/rudder) are legitimate extensions. → a new
pilot goes in `src/agents/<name>/`, **not** in the core.

## The three Unity repos (`naval-group/…`)

| Repo | Role |
|---|---|
| `LOTUSim-Unity-modules` | the main Unity project (scenes, `lotusim_interface`). |
| `LOTUSim-Unity-ros-tcp-endpoint` | the bridge receiver (fork of Unity Robotics ROS-TCP-Endpoint, ros2). Build: `colcon build --merge-install --packages-select ros_tcp_endpoint`. |
| `LOTUSim-Unity-custom-hdrp` | HDRP fork (sea/physics rendering). In production the manifest may point to the HDRP registry rather than this fork. |

## Data flow: gz ↔ xdyn ↔ Unity

A vessel has three data flows. Knowing them avoids silent orientation bugs and
"connected but nothing renders".

```
              commands (out)                 state (in)               render (out)
ROS2 vessel_cmd_array ─► gz physics_engine ─► xdyn ─► gz entity pose ─► render_plugin ─► Unity
   (thrusters / control surfaces)  (websocket)      (quaternion NED→ENU)  (ROS2/TCPUDP)   (Z→-Y)
```

### Commanding a vessel (commands → xdyn)

xdyn **ignores** the yaml `commands:` block. Setpoints are **published** on
`/<world>/vessel_cmd_array` (`lotusim_msgs/msg/VesselCmdArray`); each
`VesselCmd.cmd_string` is JSON `{"<xdyn signal>": <value>}` forwarded verbatim.

- **Propellers** (`<thrusters>` in `<lotus_param>`) → signals `<name>(rpm)`,
  `<name>(P/D)`, `<name>(beta)`. With no publisher → hardcoded default `(rpm)=2.0`
  ≈ zero thrust.
- **Angle-commanded actuators** (sails, rudder, fins) are xdyn *force models*
  (`hydrodynamic polar`) commanded by their **full signal**
  `<force model>(<angle command>)`, e.g. `mainsail(sail)`, `rudder(angle)`. The
  default command template is built from `<thrusters>` only, so an angle-commanded
  force model has no default command and must be seeded before the first ROS setpoint.

### Return state (xdyn → gz) — coordinate conventions

- **xdyn = NED frame** (North-East-Down); body = X-fwd, Y-stbd, Z-down; rotation
  `Z/Y/X` `[psi, theta', phi'']` (the only supported convention).
- **Quaternion**: order `qr, qi, qj, qk` (real first), matching xdyn's wire spec.
  The receive path (`xdyn_websocket.cpp`) must rebuild them in that order
  (`Quaterniond(qr, qi, qj, qk)`) before the frame conversion; if receive and
  send disagree, a yaw comes back as roll and heading never accumulates
  (invisible at identity orientation).
- **NED → ENU**: `quatNedToEnu()` / `vecNedToEnu()` convert xdyn's NED state into
  Gazebo's right-handed **ENU (Z-up)** frame. A **separate** step, downstream of parsing.

### Rendering (gz → Unity)

Production LOTUSim rendering is **Unity**, not the gz GUI (many models have no
`<visual>` → invisible in gz). The bridge:

- **World** plugin `<plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">`
  with `<connection_protocol>` = **`ROS2`** or **`TCPUDP`** (the only accepted values).
- Per vessel, in `<lotus_param>`: `<render_interface><publish_render>true</publish_render>
  <renderer_type_name>NAME</renderer_type_name></render_interface>`.
- Over ROS2 the gz node is **namespaced by the world name** → topics
  `/<world>/renderer_cmd` (`RendererCmd`: CREATE/DELETE, QoS TRANSIENT_LOCAL+RELIABLE)
  and `/<world>/renderer_poses` (`VesselPositionArray`, each tick). On the Unity
  side, `LotusimInterface.m_namespace` **must** equal the world name (the
  "connected, nothing renders" trap).
- **The mesh does NOT travel** on the wire: the flow carries a *name* + poses.
  Unity resolves the mesh via **Addressables**
  (`Addressables.LoadAssetAsync<GameObject>(renderer_obj_name)`) → a Unity prefab
  whose Addressable address == `renderer_type_name`.
- **Latched CREATE**: gz publishes CREATE as TRANSIENT_LOCAL at spawn; if Unity
  subscribes later, re-emit CREATE (restart gz, or publish manually with
  `--qos-durability transient_local --qos-reliability reliable`).
- **Frame conversion, Unity side** (`Assets/Scripts/lotusim_interface/common.cs`,
  `CoordinateSystemUtils.GzPoseToUnityPose`): Unity is **left-handed, Y-up**;
  position `(x, z, y)`, rotation `(-x, -z, -y, w)` — the documented `Z→-Y`. A
  fixed, generic transform applied to every gz pose (independent of xdyn).
