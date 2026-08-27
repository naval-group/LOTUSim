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

Recognised `interface_type`/`connection_type` values: `XDYNWEBSOCKET` → `XdynWebSocket`, `ROS2` → `ROS2Interface` (case-insensitive). Anything else fails with an "unknown interface_type" error and physics is disabled for that vessel/domain.

---

## `XdynWebsocket` (default interface)

A **singleton** WebSocket client connecting to xdyn - one shared client instance handles every vessel's connections, keyed internally by Gazebo entity.

**Coordinate conversion:** xdyn communicates in NED; Gazebo uses ENU. Every message is converted both ways (`vecNedToEnu`/`vecEnuToNed`, `quatNedToEnu`/`quatEnuToNed`) using a fixed basis-change quaternion.

**Per-tick protocol** (`getNewState`): builds a co-simulation request - `Dt` (converted from milliseconds to seconds), one `states` entry (the vessel's current pose/velocity converted to NED), and `commands` (pulled from the shared command map, populated by `<thrusters>` config or later overridden via ROS commands). The reply's `z` value is used to infer which domain the vessel *should* be in next: `z ≥ 10` → Aerial, `z ≤ -10` → Underwater, otherwise Surface (`z` is NED, so positive is down - this is why deep values map to Underwater and negative/high values map to Aerial).

**Default thruster commands on spawn:** every declared `<thruster>` starts at **`rpm: 50.0`, P/D: `0.79`, beta: `0.0`**.

---

## `ROS2Interface`

A second **singleton** interface, intended for bridging to an externally-driven physics source over ROS 2 rather than xdyn — subscribes to `<namespace>/poses` (`VesselPositionArray`) and looks up a vessel's pose by name each tick. Runs its own `MultiThreadedExecutor` on a dedicated thread (node name: `physics_aerial_linker`).