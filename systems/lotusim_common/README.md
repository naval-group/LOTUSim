# LOTUSim Common

`lotusim_common` is a shared utility library used across LOTUSim's Gazebo plugins - Gazebo/SDF helper functions, a lightweight entity-grouping structure, and a spdlog-based logging wrapper. It's a plain library other subsystems link against.

Used by: `lotusim_mas`, `physics_engine_interface`, `render_interface`, and the sensor plugins.

---

## Components

```
lotusim_common
    ├── common.{hpp,cpp}        — ECM/SDF helper functions (namespace lotusim::common)
    ├── entity_group.{hpp,cpp}  — EntityGraph: union-find for linked entities (namespace lotusim::common)
    └── logger.{hpp,cpp}        — spdlog console/file logger setup (namespace lotusim::logger)
```

---

## `common.hpp` / `common.cpp`

General-purpose helpers for working with Gazebo's `EntityComponentManager` (ECM) and SDF, used throughout the codebase to avoid re-implementing the same lookups in every plugin.

| Function | Description |
|---|---|
| `pose3Eql(a, b)` | Compares two `gz::math::Pose3d` for equality within a `1e-6` tolerance (position and each quaternion component) - use instead of exact float equality when checking if a pose changed |
| `getWorldName(ecm)` | Returns the name of the (first) entity with both `Name` and `World` components |
| `getModelName(ecm, entity)` | Walks up the `ParentEntity` chain from `entity` until it finds an ancestor with a `Model` component, returning that entity + its name. Use this to resolve "which vessel does this sensor/link belong to" from a child entity |
| `generateHeaderMessage(time)` | Builds a `std_msgs::msg::Header` from a `steady_clock::duration` (splits into `sec`/`nanosec`). **`frame_id` is hardcoded to `"wgs84"`** regardless of the actual frame |
| `XYFromLatLong(ecm, lat, lon)` | Converts a lat/lon pair to local `(x, y)` using the world's `SphericalCoordinates` origin. Returns `std::nullopt` if the world has no spherical coordinates configured |
| `getElementCaseInsensitive(parent, name)` | Finds a child SDF element by name, case-insensitively |
| `toUpper(str)` | Simple uppercase conversion, used internally by `getElementCaseInsensitive` |
| `shuffleOrder<T>(vector, type)` | Shuffles a `std::vector<T>` in place via `std::shuffle` |

---

## `entity_group.hpp` / `entity_group.cpp` - `EntityGraph`

A [union-find (disjoint-set)](https://en.wikipedia.org/wiki/Disjoint-set_data_structure) structure for tracking which entities are "linked" to each other, keyed by `uint64_t` entity ID.

**Real usage:** the sensor plugin (`lotusim_sensor_plugin`) holds an `EntityGraph m_collision_graph` member, used to group entities that are in contact with each other, so a compound collision event (multiple linked entities touching) can be reported as one group rather than N separate pairs.

| Method | Description |
|---|---|
| `addPair(entity1, entity2)` | Links two entities into the same group (union) |
| `areLinked(entity1, entity2)` | True if both entities are in the same group (find + compare) |
| `getGroup(entity)` | All entities in the same group as `entity` |
| `getAllSets()` | Every group currently tracked, as a list of groups |
| `getSetCount()` | Number of distinct groups |
| `clearGraph()` | Resets all tracked links |

---

## `logger.hpp` / `logger.cpp`

Thin wrapper around [spdlog](https://github.com/gabime/spdlog) providing consistent console + file logging setup across all LOTUSim plugins.

| Function | Description |
|---|---|
| `createConsoleAndFileLogger(name, file_name)` | Creates a logger writing to both stdout and a file. If a logger with `name` already exists, returns the existing one instead of creating a duplicate |
| `createBasicFileLogger(name, file_name)` | File-only variant of the above (no console sink) |
| `getLogger(name)` | Looks up an already-registered logger by name (thin wrapper over `spdlog::get`) |
| `createOrGetLogFolderPath()` | Resolves (and creates, if needed) the log output directory |
| `setLoggerPattern(logger)` | Sets the log line format based on the logger's level |
| `getLogLevelFromEnv()` | Reads the desired log level from an environment variable |

### Log file location

Logs are written under `$LOTUSIM_PATH/lotus_logs/` (falling back to the system temp directory if `LOTUSIM_PATH` isn't set). Inside that:
- If the effective log level is `info` or above: a **timestamped subfolder** (`YYYY-MM-DD_HH-MM-SS`) is created per run
- If the effective log level is below `info` (i.e. `debug`/`trace`): logs go into a folder simply named `debug` instead

Log files are created with `owner + group` read/write/execute permissions where the filesystem allows it (failures to set permissions are logged as a warning, not fatal). Files opened via `createConsoleAndFileLogger`/`createBasicFileLogger` **overwrite** any existing file of the same name (`truncate = true`), rather than appending.

### Log level

Default level is `trace` in debug builds (`DEBUG` defined) and `info` otherwise.

Accepted values (case-insensitive): `TRACE`, `DEBUG`, `INFO`, `WARN`/`WARNING`, `ERROR`, `CRITICAL`, `OFF`.

### Log line format

Set automatically based on level:
- Level `< info` (i.e. debug/trace): pattern `%v` - message only, no timestamp/level prefix (useful for high-frequency debug output)
- Level `>= info`: pattern `[%D %T] [%l]: %v` - date, time, level, then message