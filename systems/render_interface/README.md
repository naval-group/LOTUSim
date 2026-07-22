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
│    collect {vessel_name, pose} for every tracked vessel              │
│    ──► sendPosition()                                                │
│                                                                      │
│                  ┌────────────────────────────────┐                  │
│                  │      RenderInterfaceBase       │ (one instance)   │
│                  │   ◄── TcpUdpInterface          │                  │
│                  │   ◄── ROSInterface             │                  │
│                  └────────────────────────────────┘                  │
└──────────────────────────────────────────────────────────────────────┘
```

Only vessels whose SDF sets `publish_render` to `true` under `<lotus_param><render_interface>` are tracked. Everything else is ignored by this plugin.

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
  - `EachNew<ModelSdf, ParentEntity>`: for every newly spawned model, checks whether `<lotus_param><render_interface><publish_render>` is present and `true`. If so, registers the vessel entity and calls `createVessel()`.
  - `EachRemoved<ModelSdf, ParentEntity>`: for any tracked vessel that disappears, calls `destroyVessel()` and stops tracking it.
- **`PostUpdate()`** - builds a `{vessel_name, pose}` list for every tracked vessel, calls `sendPosition()`, then `customUpdates()`.

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
| `sendPosition(runTime, poses)` | Push the current pose of every tracked vessel |
| `createVessel(name, pose, sdfptr)` | Notify the renderer a vessel was spawned |
| `destroyVessel(name)` | Notify the renderer a vessel was removed |
| `customPreUpdates(info, ecm)` | Optional hook run every `PreUpdate` tick (default: no-op) |
| `customUpdates(info, ecm)` | Optional hook run every `PostUpdate` tick (default: no-op) |

---

### `TcpUdpInterface`

Talks to an external renderer over raw sockets: **UDP** for per-tick pose broadcasts, **TCP** for create/destroy commands. Every TCP command is synchronous - it blocks until it reads a `\r`-terminated response, and only returns `true` if that response is exactly `"ACK"`.

**Pose broadcast (UDP, sent every tick):**

```json
{
  "time": 12.34,
  "VesselsInfo": [
    {
      "name": "lrauv_0",
      "position": {"x": 0, "y": 0, "z": 0},
      "rotation": {"x": 0, "y": 0, "z": 0, "w": 1},
      "thrusters": []
    }
  ]
}
```

**Create command (TCP, on vessel spawn):**

```json
{
  "cmd": "create",
  "name": "lrauv_0",
  "pose": {"x": 0, "y": 0, "z": 0},
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

Talks to the renderer over ROS 2 topics instead of raw sockets. Publishers use `TRANSIENT_LOCAL` durability, so a renderer that connects after vessels have already spawned still receives the last create/destroy commands on subscribe.

**Published topics:**

| Topic | Type | Published |
|---|---|---|
| `renderer_cmd` | `lotusim_msgs/msg/RendererCmd` | On vessel create/destroy (`CREATE_CMD` / `DELETE_CMD`) |
| `renderer_poses` | `lotusim_msgs/msg/VesselPositionArray` | Every `PostUpdate` tick |

`configureInterface()` ignores the plugin SDF entirely - the connection is implicit via ROS 2 discovery, so `ip` / `udp_port` / `tcp_port` only apply when `connection_protocol` is `TCPUDP`.

**`RendererCmd.msg` command types:**

| Constant | Value | Meaning |
|---|---|---|
| `CREATE_CMD` | 0 | Spawn a vessel - fills `renderer_obj_name`, `vessel_name`, `vessel_position` |
| `DELETE_CMD` | 1 | Remove a vessel - fills `vessel_name` only |
| `EXPLODE_CMD` | 2 | Explosion effect - fills `vessel_name` only |

---

## Adding a New Render Interface

1. Create a subclass of `RenderInterfaceBase`.
2. Implement `configureInterface()`, `sendPosition()`, `createVessel()`, `destroyVessel()`. Override `customPreUpdates()` / `customUpdates()` only if you need extra per-tick work.
3. Add a branch for your protocol name in `CreateRenderInterface()` (`render_plugin.hpp`).
4. Add the new `.cpp` to `CMakeLists.txt`.