# Render Interface

The `render_interface` (built as `render_plugin`) is a Gazebo world plugin that forwards vessel spawn/destroy events and per-tick poses to an external renderer (e.g. the Unity front-end). It supports two interchangeable transports, a raw TCP/UDP protocol and ROS 2 topics, selected at configure time.

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                RenderPlugin : Gazebo World Plugin                    │
│          (one instance for the entire simulation world)              │
│                                                                      │
│  PreUpdate                                                           │
│    EachNew<ModelSdf>     ──► vessel has <render_interface>           │
│                               <publish_render>true ──► createVessel  │
│    EachRemoved<ModelSdf> ──► tracked vessel removed ──► destroyVessel│
│                                                                      │
│  PostUpdate                                                          │
│    collect {vessel_name, GeoPose} for every tracked vessel           │
│    ──► sendWorldState()                                              │
│                                                                      │
│                  ┌────────────────────────────────┐                  │
│                  │      RenderInterfaceBase       │ (one instance)   │
│                  │   ◄── TcpUdpInterface          │                  │
│                  │   ◄── ROSInterface             │                  │
│                  └────────────────────────────────┘                  │
└──────────────────────────────────────────────────────────────────────┘
```

Only vessels whose SDF sets `publish_render` to `true` under `<lotus_param><render_interface>` are tracked. Everything else is ignored by this plugin.

### Poses sent to the renderer

The renderer never receives Gazebo's local Cartesian coordinates. Every pose is a `lotusim::common::GeoPose` (`lotusim_common/geo_types.hpp`):

- **Position**: WGS84 `latitude` and `longitude` in degrees, `altitude` in metres from sea level (positive up, negative when submerged). It is converted from the vessel's world pose with `lotusim::common::worldGeoPose` (`localToGeo` underneath), the same conversion MAS uses for `VesselPosition`, so the renderer and MAS report the same position for a vessel.
- **Orientation**: the world-frame attitude quaternion, ENU (x East, y North, z Up) with body FLU, as in `geographic_msgs/GeoPose`.

The world must declare `<spherical_coordinates>`. Without it, vessels are not created in the renderer and no poses are sent; the plugin logs this.

---

## Class Hierarchy

```
RenderPlugin  (Gazebo world System plugin)
        │
        └── owns one RenderInterfaceBase   (abstract)
                    │
                    ├── TcpUdpInterface     (raw TCP/UDP JSON protocol)
                    ├── ROSInterface        (ROS 2 topics)
                    └── users implementation
```

---

## Components

### `RenderPlugin` : World Plugin

The top-level Gazebo system plugin. Declared once in the world SDF.

- **`Configure()`** - reads `<connection_protocol>` from the plugin SDF, builds the matching `RenderInterfaceBase` via `CreateRenderInterface()`, and calls `configureInterface()` on it.
- **`PreUpdate()`**
  - Calls `customPreUpdates()` on the active interface.
  - `EachNew<ModelSdf, ParentEntity>`: for every newly spawned model, checks whether `<lotus_param><render_interface><publish_render>` is present and `true`. If so, registers the vessel entity and calls `createVessel()` with its spawn `GeoPose`.
  - `EachRemoved<ModelSdf, ParentEntity>`: for any tracked vessel that disappears, calls `destroyVessel()` and stops tracking it.
- **`PostUpdate()`** - builds a `{vessel_name, GeoPose}` list for every tracked vessel, calls `sendWorldState()`, then `customUpdates()`.

**World SDF declaration:**

```xml
<plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">
    <connection_protocol>TCPUDP</connection_protocol>  <!-- or ROS2 -->
    <ip>127.0.0.1</ip>
    <udp_port>23456</udp_port>
    <tcp_port>23457</tcp_port>
</plugin>
```

**Per-vessel SDF tags**, inside the vessel's `<lotus_param>` block:

```xml
<lotus_param>
    <render_interface>
        <publish_render>true</publish_render>
        <renderer_type_name>lrauv</renderer_type_name>
    </render_interface>
</lotus_param>
```

`renderer_type_name` is forwarded as the vessel's `type` on creation, so the renderer knows which asset to instantiate.

---

### `RenderInterfaceBase` : Abstract Interface

Defines the contract every renderer connection must implement:

| Method | Description |
|---|---|
| `configureInterface(sdf)` | Parse plugin-level SDF params (ip, ports ...) |
| `sendWorldState(runTime, poses)` | Push the current `GeoPose` of every tracked vessel |
| `createVessel(name, pose, sdfptr)` | Notify the renderer a vessel was spawned at `pose` (`GeoPose`) |
| `destroyVessel(name)` | Notify the renderer a vessel was removed |
| `customPreUpdates(info, ecm)` | Optional hook run every `PreUpdate` tick (default: no-op) |
| `customUpdates(info, ecm)` | Optional hook run every `PostUpdate` tick (default: no-op) |

---

### `TcpUdpInterface`

Talks to an external renderer over raw sockets: **UDP** for per-tick pose broadcasts, **TCP** for create/destroy commands. Every TCP command is synchronous - it blocks until it reads a `\r`-terminated response, and only returns `true` if that response is exactly `"ACK"`.

**World state (UDP, every PostUpdate tick, capped by `render_rate_hz`):** the same layout as `lotusim_msgs/RendererState`. Each datagram is the full set of live vessels; a vessel missing from it has been removed.

```json
{
  "header": {"stamp": {"sec": 12, "nanosec": 340000000}, "frame_id": "world"},
  "vessels": [
    {
      "vessel_name": "lrauv_0",
      "renderer_obj_name": "lrauv",
      "vessel_pose": {
        "position": {"latitude": 43.1, "longitude": 5.9, "altitude": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
      }
    }
  ]
}
```

**Create command (TCP, on vessel spawn):**

```json
{
  "cmd": "create",
  "name": "lrauv_0",
  "pose": {"latitude": 43.1, "longitude": 5.9, "altitude": 0},
  "type": "lrauv"
}
```

**Destroy command (TCP, on vessel removal):**

```json
{
  "cmd": "destroy",
  "name": "lrauv_0"
}
```

**SDF parameters:**

| Parameter | Default | Description |
|---|---|---|
| `ip` | `127.0.0.1` | Renderer host |
| `udp_port` | `23456` | Pose broadcast port |
| `tcp_port` | `23457` | Create/destroy command port |

---

### `ROSInterface`

Talks to the renderer over ROS 2 topics instead of raw sockets. Every `sendWorldState()` publishes one snapshot on `renderer_state` (`KeepLast(1)`, `TRANSIENT_LOCAL`) holding every live vessel and its current pose, so a renderer that connects late receives the current world in a single message. The rate follows `PostUpdate`, capped by `render_rate_hz`; a snapshot is sent even when no vessel is left. `renderer_cmd` is a plain event stream (`VOLATILE`, `KeepLast(10)`) for one-off events such as explosions.

**Published topics:**

| Topic | Type | Published |
|---|---|---|
| `renderer_state` | `lotusim_msgs/msg/RendererState` | Every `PostUpdate` tick, capped by `render_rate_hz` (latched) |
| `renderer_cmd` | `lotusim_msgs/msg/RendererCmd` | One-off events (`EXPLODE_CMD`) |

`configureInterface()` ignores the plugin SDF entirely - the connection is implicit via ROS 2 discovery, so `ip` / `udp_port` / `tcp_port` only apply when `connection_protocol` is `TCPUDP`.

**`RendererState.msg`:** `header` plus `RendererVessel[] vessels`, where each `RendererVessel` has `vessel_name`, `renderer_obj_name` and `vessel_pose` (`geographic_msgs/GeoPose`, the vessel's current pose). Each message replaces the previous one: the renderer spawns vessels it does not have yet, moves the ones it has, and removes the ones that are no longer listed.

**`RendererCmd.msg` command types:**

| Constant | Value | Meaning |
|---|---|---|
| `CREATE_CMD` | 0 | Not sent by `ROSInterface`; vessel creation is carried by `renderer_state` |
| `DELETE_CMD` | 1 | Not sent by `ROSInterface`; vessel deletion is carried by `renderer_state` |
| `EXPLODE_CMD` | 2 | Explosion effect - fills `vessel_name` only |

---

## Adding a New Render Interface

1. Create a subclass of `RenderInterfaceBase`.
2. Implement `configureInterface()`, `sendWorldState()`, `createVessel()`, `destroyVessel()`. Override `customPreUpdates()` / `customUpdates()` only if you need extra per-tick work.
3. Add a branch for your protocol name in `CreateRenderInterface()` (`render_plugin.hpp`).
4. Add the new `.cpp` to `CMakeLists.txt`.