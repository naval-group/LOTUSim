# Entity Manager

The `entity_manager` plugin is the core of LOTUSim's Multi-Agent System - it's the single Gazebo world plugin responsible for spawning, moving, and deleting simulated vessels on command, and for publishing every vessel's position each tick. It's designed to be subclassed for scenario-specific behavior via a small set of `customUser*` extension points.

---

## World SDF Declaration

```xml
<plugin filename="entity_manager_plugin" name="lotusim::gazebo::EntityManager">
</plugin>
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                  EntityManager : Gazebo World Plugin                │
│              (one instance for the entire simulation)               │
│                                                                     │
│  Configure() ─ sets up ROS node, action servers, publisher          │
│                                                                     │
│  Action goals accepted ──► queued (m_mas_cmds / m_mas_cmds_array)   │
│                                    │                                │
│  PreUpdate()  ── drains queue, shuffles order, dispatches           │
│                     via handleMASCmd() → addEntity/moveEntity/      │
│                     deleteEntity() → customUserAddEntity/           │
│                     customUserDeleteEntity()                        │
│                                                                     │
│  Update()     ── EachNew<ModelSdf>: registers newly spawned vessel  │
│                     in m_vessels_entities / m_vessels_names,        │
│                     enables velocity checks on its base_link        │
│                ── EachRemoved<ModelSdf>: deregisters a deleted      │
│                     vessel                                          │
│                                                                     │
│  PostUpdate() ── publishPose(): publishes every tracked vessel's    │
│                     pose + geo-position to `poses`                  │
│                     customUserPostUpdate()                          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ROS 2 Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `poses` | Publisher (`lotusim_msgs/msg/VesselPositionArray`) | Every tracked vessel's pose (position + orientation) and geographic position (lat/lon/elevation), published every `PostUpdate` |
| `mas_cmd` | Action server (`lotusim_msgs/action/MASCmd`) | Single-command spawn/move/delete requests |
| `mas_cmd_array` | Action server (`lotusim_msgs/action/MASCmdArray`) | Batched multi-command requests - processed as one unit, with per-command results returned together |

Both action servers currently **reject cancellation** (`CancelResponse::REJECT`) - once a goal is accepted, it will run to completion.

`mas_cmd_array` uses default QoS/action-server options. `mas_cmd` uses a **custom QoS profile**: `KEEP_LAST` history with depth 20 (vs. the ROS 2 default of 10), reliable delivery, volatile durability. If you're sending commands faster than they're drained, `mas_cmd`'s larger queue gives you more buffer before goals start being dropped.

The plugin runs its own `rclcpp::Node` (`gz_entity_management_node`, namespaced to the Gazebo world) on a dedicated thread with a `MultiThreadedExecutor`, using separate mutually-exclusive callback groups per action server.

---

## Command Processing Pipeline

Commands are queued (`handleMASCmd*Accepted`, under a mutex) and drained once per tick in `PreUpdate()`. Before processing, the queued batch is **randomly shuffled** (via `lotusim::common::shuffleOrder`), this avoids any systematic ordering bias (e.g. always favoring whichever vessel's commands happened to queue first) when multiple agents submit commands in the same tick.

Each command dispatches by `cmd_type` in `handleMASCmd()`:

### `CREATE_CMD` → `addEntity()`

Two ways to specify what to spawn:

1. **`model_name` set** - loads `$LOTUSIM_MODELS_PATH/<model_name>/<sdf_file>` (default filename: **`model.sdf`**, confirming the naming convention from [Extend with your components](extend-with-your-components#adding-models-into-lotusim)), then merges the `lotus_param` block from `msg.sdf_string` into the loaded model's SDF via direct XML manipulation (tinyxml2) before parsing.
2. **`model_name` empty** - `msg.sdf_string` is expected to be a complete, standalone SDF model description.

Fails (returns `std::nullopt`) if `LOTUSIM_MODELS_PATH` isn't set, the SDF file/elements can't be found, or the SDF fails to parse.

**Name collision handling:** if the desired vessel name (from `msg.vessel_name`, or the model's own name if unset) is already in use in the world, a numeric suffix is appended and incremented until a free name is found (`name`, `name_0`, `name_1`, ...), this is what allows spawning multiple instances of the same model.

On success, the new entity is immediately moved to its requested position via an internal `MOVE_CMD`.

### `MOVE_CMD` → `moveEntity()`

Resolves the target vessel by `entity` ID if provided, otherwise by `vessel_name` lookup. Position can be given directly (`vessel_position`) or as a geographic point (`geo_point`). If `geo_point` has a non-zero latitude or longitude, it's converted to local XY via `lotusim::common::LatLongToXY` and overrides the X/Y from `vessel_position` (Z and orientation always come from `vessel_position`).

---

## Extension Points

Subclass `EntityManager` and override any of:

| Method | Called |
|---|---|
| `customUserConfiguration(sdf)` | End of `Configure()` |
| `customUserPreUpdate()` | End of `PreUpdate()`, after all queued commands are processed |
| `customUserPostUpdate()` | End of `PostUpdate()`, after poses are published |
| `customUserAddEntity(msg)` | After a successful `CREATE_CMD` |
| `customUserDeleteEntity(msg)` | After a successful `DELETE_CMD` |

All five have empty default implementations - safe to override only the ones you need.

**Real example:** `aerial_demo_entity_manager`'s `AerialMAS` class overrides `customUserConfiguration`, `customUserAddEntity`, and `customUserDeleteEntity` to forward spawn/delete events to a separate aerial-specific MAS action client (`m_aerial_mas_client`).
