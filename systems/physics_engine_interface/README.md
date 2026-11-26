# Physics Engine Interface

`physics_engine_interface` is the Gazebo world plugin that connects simulated vessels to external physics engines (xdyn, or a custom ROS 2 source) sending state each tick and applying the returned pose/velocity back onto the Gazebo model. It supports vessels transitioning between **aerial**, **surface**, and **underwater** domains, each of which can be backed by a different physics server.

---

## World SDF Declaration

```xml
<plugin filename="physics_interface_plugin" name="lotusim::gazebo::PhysicsInterfacePlugin">
</plugin>
```

---

## Architecture Overview

```
┌───────────────────────────────────────────────────────────────────────┐
│            PhysicsInterfacePlugin : Gazebo World Plugin               │
│                                                                       │
│  Update() ─ EachNew<ModelSdf>  → loadVessel()                         │
│           ─ EachRemoved<ModelSdf> → deleteVessel()                    │
│           ─ per vessel, in parallel: std::async → updateVesselState() │
│                                        → getNewState() on the         │
│                                          vessel's current interface   │
│                                        → vesselDomainTransition() if  │
│                                          domain changed               │
│           ─ waits on every future before the tick completes           │
│                                                                       │
│  Per vessel: up to 3 PhysicsInterfaceBase instances tracked           │
│  (m_aerial_interface / m_surface_interface / m_underwater_interface), │
│  one active at a time (m_current_vessel_interface)                    │
└───────────────────────────────────────────────────────────────────────┘
                                │
                 PhysicsInterfaceBase (abstract)
                   ├── XdynWebsocket   (singleton, default)
                   └── ROS2Interface   (singleton, aerial-linking use case)
```

Only models with a `physics_engine_interface` element inside `lotus_param` are tracked, everything else is ignored.

---

## SDF Configuration

```xml
<model name="drone">
  <lotus_param>
    <physics_engine_interface>
      <aerial>
        <interface_type>XDynWebSocket</interface_type>
        <uri>127.0.0.1:1234</uri>
        <thrusters>
          <thruster1>propeller1</thruster1>
        </thrusters>
      </aerial>
      <surface>
        <interface_type>ROS2</interface_type>
        <uri>127.0.0.1:1235</uri>
      </surface>
      <underwater>
        <interface_type>XDynWebSocket</interface_type>
        <uri>127.0.0.1:1236</uri>
      </underwater>
      <init_state>Surface</init_state>
    </physics_engine_interface>
  </lotus_param>
</model>
```

Define only the domains your vessel actually needs (e.g. a submarine only needs `underwater` + `surface`). `init_state` sets which domain is active at spawn - required; loading fails if it's missing or unrecognised.

> ℹ️ **`interface_type` vs. `connection_type`:** the plugin accepts **both** tag names - `connection_type` is explicitly marked in source as the legacy tag, kept for backward compatibility, with `interface_type` as the current name.

Recognised `interface_type`/`connection_type` values: `XDYNWEBSOCKET` → `XdynWebSocket`, `ROS2` → `ROS2Interface`, `FMU` → `FmuInterface` (case-insensitive). Anything else fails with an "unknown interface_type" error and physics is disabled for that vessel/domain.

---

## `XdynWebsocket` (default interface)

A **singleton** WebSocket client connecting to xdyn - one shared client instance handles every vessel's connections, keyed internally by Gazebo entity.

**Coordinate conversion:** xdyn communicates in NED with velocities expressed in body frame; Gazebo uses ENU with world velocities. Class `VesselInformation` is used to perform conversion.

**Per-tick protocol** (`getNewState`): builds a co-simulation request - `Dt` (converted from milliseconds to seconds), one `states` entry (the vessel's current pose/velocity converted to NED), and `commands` (pulled from the shared command map, populated by `<thrusters>` config or later overridden via ROS commands). The reply's `z` value is used to infer which domain the vessel *should* be in next: `z ≥ 10` → Aerial, `z ≤ -10` → Underwater, otherwise Surface (`z` is NED, so positive is down - this is why deep values map to Underwater and negative/high values map to Aerial).

**Default thruster commands on spawn:** every declared `<thruster>` starts at **`rpm: 50.0`, P/D: `0.79`, beta: `0.0`**.

---

## `FmuInterface`

Computes the vessel's physics with an **FMI 2.0 Co-Simulation FMU** loaded in the simulator's process: no server to start, no port, no message per step. Each tick is one `fmi2DoStep(t, Dt)`, the way `XdynWebsocket` sends one request to xdyn.

```xml
<surface>
  <interface_type>FMU</interface_type>
  <fmu>dtmb_hull/dtmb.fmu</fmu>
  <fmu_parameters>
    <parameter name="xdyn.dt">0.2</parameter>
  </fmu_parameters>
</surface>
```

From a scenario: `interface_type: FMU`, then `interface_params: {fmu: dtmb_hull/dtmb.fmu, fmu_parameters: {xdyn.dt: 0.2}}`.

- **`<fmu>`**: absolute, or relative to `LOTUSIM_MODELS_PATH`, then to each `GZ_SIM_RESOURCE_PATH` directory.
- **`<fmu_parameters>`** (optional): set before initialization, any FMI type, values as text; an Enumeration by item name (`<parameter name="xdyn.solver">rk4</parameter>`).

**The FMU's variables** follow lxdyn's FMU (`xdyn_wrapper_fmi`), so an xdyn model packed with its `make_fmu.py` fits as is. `<body>` is the body's name in the xdyn YAML, found as the one prefix under which all 13 states are outputs:

| Variables | Causality | |
|---|---|---|
| `<body>.x` … `<body>.qk` (`x y z u v w p q r qr qi qj qk`) | output (Real), required | NED position, FRD body velocities and rates, attitude quaternion |
| `<body>.reset_state`, `<body>.reset_x` … `<body>.reset_qk` | input, optional | when present, Gazebo's state is imposed before every step (`reset_state` set to false then true, the FMU resetting on that edge): teleports and domain changes carry over, as with xdyn-for-cs. Without them, the FMU keeps its own state (a warning says so) |
| the input whose description is the vessel's command, e.g. `<body>.PSPropRudd.rpm` for `PSPropRudd(rpm)`; or the input of that name | input (Real) | from the shared command map. `(rpm)` commands are converted to rad/s, as for xdyn. A command the FMU lacks is ignored, with one warning |

The domain for the next tick follows the FMU's altitude, with the same thresholds as `XdynWebsocket`.

**Instances.** There is one instance per vessel and domain. Each one unpacks the `.fmu` into its own temporary directory and loads its own copy of the library, so instances share no state and are stepped in parallel. The directory is removed with the instance.

**Platforms and dependencies.** Linux 64-bit binaries are used (`binaries/linux64`). `libzip` unpacks the archive and `tinyxml2` reads `modelDescription.xml`. The FMI 2.0 headers are in `third_party/fmi2` (BSD-2-Clause, Modelica Association).

---

## `ROS2Interface`

A second **singleton** interface, intended for bridging to an externally-driven physics source over ROS 2 rather than xdyn — subscribes to `<namespace>/poses` (`VesselPositionArray`) and looks up a vessel's pose by name each tick. Runs its own `MultiThreadedExecutor` on a dedicated thread (node name: `physics_aerial_linker`).