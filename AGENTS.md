# AGENTS.md — LOTUSim

Orientation for coding agents (and new human contributors) working in this
repository. LOTUSim is an open-source (EPL-2.0) multi-agent naval simulator:
ROS2 + Gazebo for orchestration and rendering, xdyn for rigid-body dynamics,
coupled in co-simulation.

## Read this first: physics is an external server

Physics is not a Gazebo plugin — it is an external `xdyn` process. The Gazebo
`physics_engine_interface` is a websocket client that connects to an
`xdyn-for-cs` server (one per vessel, on a TCP port). `lotusim run` launches
only `gz`, not xdyn. Without an xdyn server listening, a vessel spawns and
renders but has zero dynamics (log: `XdynWebsocket::onFail ... Removing physics`).

## Environment

- Ubuntu 24.04 → ROS2 Jazzy + Gazebo Harmonic (prefix `gz`, not `ign`).
- ROS setup scripts assume bash; under zsh, source the `.zsh` variant (`setup.zsh`) or use the `lotusim` wrapper.

Two ways to get a working stack — the container is the reproducible one.

### Container (recommended)

A prebuilt image ships the full ROS2 + Gazebo + built LOTUSim workspace:

    docker pull ghcr.io/naval-group/lotusim:latest   # release also tags :<short-sha>
    docker run --rm -it ghcr.io/naval-group/lotusim:latest bash

Build/test a local change against the image's prebuilt workspace — mount your
source, rebuild only the affected package:

    docker run --rm -v "$PWD":/lotusim_ws/src/LOTUSim ghcr.io/naval-group/lotusim:latest \
      bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source /lotusim_ws/install/setup.bash \
               && cd /lotusim_ws && colcon build --merge-install --packages-select <pkg>'

Headless (no GUI) — use it for builds, tests, and headless runs.

### Native

    lotusim install        # one-time: deps + build
    lotusim build          # rebuild workspace (colcon, --merge-install); clean_build to start over
    colcon build --merge-install --packages-select <pkg>   # single package

## Run a world

- With physics: start one `xdyn-for-cs` per vessel (each on its port), then
  `lotusim run <world>.world`. Success log:
  `PhysicsInterfacePlugin::loadVessel: ... Surface init completed` + `onOpen`.
- Without physics (render / plugin debug): `lotusim run --gui <world>.world`.

## Move a vessel under physics

Thrust comes neither from the yaml `commands:` block nor from `waypoint_follower`
(a separate kinematic mode). Publish `lotusim_msgs/msg/VesselCmdArray` on
`/<world>/vessel_cmd_array`; each `VesselCmd.cmd_string` is JSON
`{"<thruster>(rpm)": <val>, ...}` forwarded to xdyn. Reference controllers live
in the `LOTUSim-generic-scenario` repo, not in this core.

## Add a vehicle (the typical contribution)

A model lives in `assets/models/<name>/`: `model.config`, `model.sdf` (SDF;
version varies by model, 1.6–1.10), an xdyn model file (naming varies:
`wamv.yaml`, `dtmb-xdyn.yml`, `fremmConfig.yaml`, …) holding the dynamics
(inertia, added mass, damping, propulsion), and `meshes/`. Wire it into a world
via `<include>` + `<lotus_param>`.
The model (assets) goes in this core; its controller/scenario goes in
`LOTUSim-generic-scenario`. Deep dive: `.agents/skills/lotusim-developer/references/models-and-worlds.md`.

## Coordinate conventions

xdyn is NED, right-handed; Gazebo is ENU. Quaternion wire order is
`qr, qi, qj, qk` (= w, x, y, z), by name. Gazebo↔Unity applies a `Z → -Y`
transform on the Unity side.

## Contributing

Issue (with the appropriate label) → announce yourself → fork → implement →
test → PR referencing the issue (see `CONTRIBUTING.md`). EPL-2.0: never vendor
GPL or non-redistributable assets. Whoever opens the PR answers for every line,
AI-assisted or not; disclosing AI assistance (e.g. an `Assisted-by:` commit
trailer) is good practice, though the project has no formal policy on it yet.

## Related repositories (naval-group)

LOTUSim spans several repos — this core is only the simulation backend.

- **LOTUSim** — this repo: the simulator core (ROS2 + Gazebo + xdyn co-sim).
- **LOTUSim-Xdyn** — the xdyn physics engine, a fork/mirror of the canonical
  upstream at `gitlab.com/sirehna_naval_group/sirehna/xdyn`. Engine fixes go to
  the **GitLab upstream**, not this GitHub mirror.
- **LOTUSim-generic-scenario** — reference controllers/scenarios (`src/agents/`),
  the auto-installer, and a prebuilt Unity player. Vessel *controllers* live here,
  not in this core.
- **LOTUSim-Unity-modules**, **LOTUSim-Unity-custom-hdrp** (HDRP fork),
  **LOTUSim-Unity-ros-tcp-endpoint** (Unity↔ROS bridge) — the Unity rendering
  frontend (an alternative to the Gazebo GUI).
- **LOTUSim-UI-frontend** (React), **LOTUSim-UI-backend** — the web UI.
- **LOTUSim-multi-agents-benchmark** — multi-agent benchmarking.

## Further reading

- **Project wiki** — authoritative human docs (xdyn User Guide, model/world
  formats, testing): https://github.com/naval-group/LOTUSim/wiki
- **Coding-agent skill** — `.agents/skills/lotusim-developer/` ships a deeper,
  task-oriented map. Its `SKILL.md` opens with 6 field-tested pitfalls; to enable
  it in your agent (Codex, Claude Code, …), see
  `.agents/skills/lotusim-developer/README.md`. Deep-dive references:
  - `references/architecture.md` — co-sim data flow (gz ↔ xdyn ↔ Unity), ecosystem repos, coordinate conventions
  - `references/models-and-worlds.md` — model/world anatomy, adding a vehicle
  - `references/run-and-verify.md` — xdyn orchestration, verifying a vessel actually moves
