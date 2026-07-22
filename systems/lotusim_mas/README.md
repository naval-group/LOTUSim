# Multi-Agent System (MAS)

The `lotusim_mas` package (built as `multi_agent_system_plugin`) is the world plugin responsible for spawning, moving, and deleting vessels at runtime, publishing their poses, and loading/running YAML scenarios. It's the backend behind the Web UI's scenario editor and "Launch Scenario" button, the `/mas_cmd` and `/mas_cmd_array` actions used by the [scripting examples](../../examples), and the `launch_scenario` / `stop_scenario` services.

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│              MultiAgentSystem : Gazebo World Plugin                │
│           (one instance for the entire simulation world)           │
│                                                                    │
│  Configure(): spin up ROS node + dedicated executor thread,        │
│               build EntitySpawner + ScenarioManager                │
│                                                                    │
│  PreUpdate(): drain queued mas_cmd / mas_cmd_array goals           │
│               (received async on the ROS executor thread)          │
│               ──► shuffle order ──► handleMASCmd() ──► reply       │
│                                                                    │
│  Update(): EachNew<ModelSdf> / EachRemoved<ModelSdf>               │
│               ──► EntitySpawner::registerNewEntity/unregisterEntity│
│               (tracks every vessel, however it was spawned)        │
│                                                                    │
│  PostUpdate(): publish poses topic                                 │
│                                                                    │
│         ┌─────────────────┐        ┌───────────────────────┐       │
│         │  EntitySpawner  │◄───────│   ScenarioManager     │       │
│         │  add/move/      │        │   launch_scenario /   │       │
│         │  delete entity  │        │   stop_scenario srv   │       │
│         └─────────────────┘        │   ──► ScenarioParser  │       │
│                                    │       (YAML → agents) │       │
│                                    └───────────────────────┘       │
└────────────────────────────────────────────────────────────────────┘
```

---

## Class Hierarchy

```
MultiAgentSystem  (Gazebo world System plugin)
    │
    ├── owns EntitySpawner    - entity lifecycle: add / move / delete
    ├── owns ScenarioManager  - YAML scenario load / launch / stop
    │         └── uses ScenarioParser  (YAML → ScenarioConfig)
    │         └── uses EntitySpawner   (to spawn each parsed agent)
    │
    └── extension points (virtual, override by subclassing):
          customUserConfiguration()
          customUserPreUpdate()
          customUserPostUpdate()
          customUserAddEntity()
          customUserDeleteEntity()
```

---

## Components

### `MultiAgentSystem` : World Plugin

Declared once in the world SDF:

```xml
<plugin filename="multi_agent_system_plugin" name="lotusim::gazebo::MultiAgentSystem">
</plugin>
```

- **`Configure()`** - creates the plugin's logger and ROS 2 node, an `SdfEntityCreator`, an `EntitySpawner`, and a `ScenarioManager`; starts a `MultiThreadedExecutor` spinning on its own thread (so action/service callbacks don't block the simulation loop); calls `customUserConfiguration()`.
- **`PreUpdate()`** - action goals arrive asynchronously on the executor thread and are queued (`m_mas_cmds` / `m_mas_cmds_array`). Each tick, the queue is drained, **shuffled into random order** (so no client can rely on execution order when multiple commands land in the same tick), processed one by one through `handleMASCmd()`, and each goal is completed with its result. Then calls `customUserPreUpdate()`.
- **`Update()`** - `EachNew<ModelSdf>` / `EachRemoved<ModelSdf>` track *every* vessel that appears or disappears in the world, not just ones spawned through MAS, and keep `EntitySpawner`'s name↔entity maps in sync via `registerNewEntity()` / `unregisterEntity()`.
- **`PostUpdate()`** - publishes `poses` (pose + geo-point for every tracked vessel), then calls `customUserPostUpdate()`.

**Published / served interfaces:**

| Interface | Type | Direction | Purpose |
|---|---|---|---|
| `poses` | Topic - `lotusim_msgs/msg/VesselPositionArray` | Publishes | Pose + geo-point of every tracked vessel, every tick |
| `mas_cmd` | Action - `lotusim_msgs/action/MASCmd` | Server | Single create / move / delete command |
| `mas_cmd_array` | Action - `lotusim_msgs/action/MASCmdArray` | Server | Batch of commands, executed together in one tick |
| `launch_scenario` | Service - `lotusim_msgs/srv/String` | Server | Load and spawn every agent in a YAML scenario file |
| `stop_scenario` | Service - `std_srvs/srv/Trigger` | Server | Despawn the currently running scenario |

**`MASCmd` message fields:**

| Field | Used for |
|---|---|
| `cmd_type` | `CREATE_CMD` (0) / `DELETE_CMD` (1) / `MOVE_CMD` (2) |
| `sdf_string` | Full SDF (create), or just a `<lotus_param>` block to inject (create with `model_name`) |
| `model_name` | Asset name under `$LOTUSIM_MODELS_PATH` (create) |
| `sdf_file` | Optional SDF filename inside the model folder - defaults to `model.sdf` |
| `vessel_name` | Target vessel - required unless `entity` is set. Must not start with a digit or contain characters other than letters, digits, `_` |
| `entity` | Target vessel by entity ID (delete / move); ignored and overwritten on create |
| `vessel_position` | Cartesian pose (create / move) |
| `geo_point` + `heading` | Geographic pose (create / move) - takes priority over `vessel_position` when non-zero |

---

### `EntitySpawner` : Entity Lifecycle

Owns the vessel name↔entity maps and exposes `addEntity()` / `moveEntity()` / `deleteEntity()` / `deleteAllEntities()`, used by both `MultiAgentSystem` (ad-hoc `MASCmd` requests) and `ScenarioManager` (bulk scenario spawn/despawn).

- **`addEntity()`** - two ways to specify what to spawn:
  - `sdf_string` set, `model_name` empty → the full SDF is loaded as-is.
  - `model_name` set → loads `$LOTUSIM_MODELS_PATH/<model_name>/<sdf_file or model.sdf>` from disk; if `sdf_string` is also set, its `<lotus_param>` block is parsed and injected into the loaded model's XML before spawning.

  The final vessel name comes from `vessel_name`, falling back to the SDF's own model name, then passed through `resolveUniqueName()` (appends `_0`, `_1`, … if already taken). On success the entity is created, parented to the world, and immediately moved to the requested pose via an internal `moveEntity()` call.
- **`moveEntity()`** - resolves the target by `entity` or `vessel_name`, then sets its `Pose` component. If a non-zero `geo_point` or `heading` is present, it's converted to world XY via the world's spherical-coordinates reference and overrides the Cartesian pose.
- **`deleteEntity()` / `deleteAllEntities()`** - remove the vessel from the internal maps immediately, then request removal via `SdfEntityCreator`; Gazebo actually removes the entity on the next update step.
- **Thread-safety** - `addEntity` / `moveEntity` / `deleteEntity` run only on the Gazebo `PreUpdate` thread, so ECM writes need no locking. The name↔entity maps themselves are protected by a `shared_mutex` since they're also read from the pose-publishing path in `PostUpdate`.

---

### `ScenarioManager` + `ScenarioParser` : YAML Scenarios

`ScenarioManager` is a plain helper owned by `MultiAgentSystem`. It exposes the `launch_scenario` / `stop_scenario` services:

- **`launch_scenario`** - looks up `$LOTUSIM_SCENARIOS_PATH/<request.data>` (falling back to `~/lotusim_ws/src/LOTUSim/assets/scenarios` with a warning if the env var isn't set), refuses if a scenario is already running, parses the YAML via `ScenarioParser`, applies `reference_position` to the world's spherical-coordinates origin, then spawns every agent via `EntitySpawner`.
- **`stop_scenario`** - despawns every agent spawned by the active scenario and clears the current-scenario state.

`ScenarioParser` is a stateless YAML → `ScenarioConfig` converter. For each agent, its `render_interface` / `physics_interface` / `waypoint_interface` YAML blocks are converted **directly** into a `<lotus_param>` XML string.

**Scenario YAML schema:**

```yaml
name: defenseScenario
time_of_day: "12:00"
date: "2026-01-01"

reference_position:
  latitude: -34.8852
  longitude: 138.6217
  altitude: 0.0

agents:
  lrauv1:
    model: lrauv
    heading: 90.0
    position:
      latitude: -34.8850
      longitude: 138.6220
      altitude: -50.0

    render_interface:
      enabled: true
      renderer_type: lrauv

    physics_interface:
      init_domain: underwater
      domains:
        - domain: underwater
          interface_type: XDynWebSocket
          interface_params:
            uri: 127.0.0.1:12346
            thrusters:
              - name: propeller

    waypoint_interface:
      enabled: true
      loop: true
      mode: circle
      circle:
        radius: 50.0
```

---

## Adding Custom MAS Behavior

`MultiAgentSystem` exposes five virtual hooks (all empty by default) for project-specific logic without having to fork the whole plugin:

| Hook | Called |
|---|---|
| `customUserConfiguration(sdf)` | End of `Configure()` - read your own plugin-level SDF params |
| `customUserPreUpdate()` | End of `PreUpdate()`, after MAS commands for this tick are processed |
| `customUserPostUpdate()` | End of `PostUpdate()`, after poses are published |
| `customUserAddEntity(msg)` | After a `CREATE_CMD` succeeds |
| `customUserDeleteEntity(msg)` | After a `DELETE_CMD` succeeds |

To use them, derive your own class and register it as the plugin instead of the base one:

```cpp
// my_mas_plugin.hpp
#include "lotusim_mas/multi_agent_system.hpp"

namespace my_project {

class MyMasPlugin : public lotusim::gazebo::MultiAgentSystem {
protected:
    void customUserConfiguration(
        const std::shared_ptr<const sdf::Element>& sdf) override
    {
        // read your own SDF params here
    }

    void customUserAddEntity(const lotusim_msgs::msg::MASCmd& msg) override
    {
        // e.g. register the new vessel with a project-specific tracker
    }
};

}  // namespace my_project

GZ_ADD_PLUGIN(
    my_project::MyMasPlugin,
    gz::sim::System,
    my_project::MyMasPlugin::ISystemConfigure,
    my_project::MyMasPlugin::ISystemPreUpdate,
    my_project::MyMasPlugin::ISystemUpdate,
    my_project::MyMasPlugin::ISystemPostUpdate)
```

Then reference your plugin's library/class in the world SDF instead of `multi_agent_system_plugin` / `lotusim::gazebo::MultiAgentSystem`.

For sending `MASCmd` / `MASCmdArray` goals from your own ROS 2 node (rather than the Web UI), see the `controlling_ships.py` walkthrough in the [Tutorial](../../Tutorial.md#manually-script-based) - it spawns a vessel with dynamics and drives it via this same action interface.