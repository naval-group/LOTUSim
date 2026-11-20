# LOTUSim Common

`lotusim_common` is a shared utility library used across LOTUSim's Gazebo plugins - Gazebo/SDF helper functions, a lightweight entity-grouping structure, and a spdlog-based logging wrapper. It's a plain library other subsystems link against.

Used by: `lotusim_mas`, `physics_engine_interface`, `render_interface`, and the sensor plugins.

---

## Components

```
lotusim_common
    ├── common.{hpp,cpp}        — ECM/SDF helper functions (namespace lotusim::common)
    ├── transforms.{hpp,cpp}    — angle, frame and geographic conversions (namespace lotusim::common)
    ├── entity_group.{hpp,cpp}  — EntityGraph: union-find for linked entities (namespace lotusim::common)
    └── logger.{hpp,cpp}        — spdlog console/file logger setup (namespace lotusim::logger)
```

---

## `common.hpp` / `common.cpp`

General-purpose helpers for working with Gazebo's `EntityComponentManager` (ECM) and SDF, used throughout the codebase to avoid re-implementing the same lookups in every plugin.

| Function | Description |
|---|---|
| `pose3Eql(a, b)` | Compares two `gz::math::Pose3d` for equality within a `1e-6` tolerance (position and each quaternion component) - use instead of exact float equality when checking if a pose changed |
| `getWorldName(ecm)` | Returns the name of the world entity (`gz::sim::worldEntity`) |
| `getModelName(ecm, entity)` | Walks up the `ParentEntity` chain from `entity` until it finds an ancestor with a `Model` component, returning that entity + its name. Use this to resolve "which vessel does this sensor/link belong to" from a child entity |
| `generateHeaderMessage(time)` | Builds a `std_msgs::msg::Header` from a `steady_clock::duration` (splits into `sec`/`nanosec`). **`frame_id` is hardcoded to `"wgs84"`** regardless of the actual frame |
| `getElementCaseInsensitive(parent, name)` | Finds a child SDF element by name, case-insensitively |
| `toUpper(str)` | Simple uppercase conversion, used internally by `getElementCaseInsensitive` |
| `shuffleOrder<T>(vector, type)` | Shuffles a `std::vector<T>` in place via `std::shuffle` |

---

## `transforms.hpp` / `transforms.cpp` - conversions

Every conversion between what LOTUSim exposes (WGS84 degrees, altitude in metres from sea level, negative when submerged, angles in degrees from true North, clockwise) and Gazebo's flat ENU world (sea surface at z = 0, yaw in radians from East, counter-clockwise), and between axis conventions. The pure functions take the world origin (`gz::math::SphericalCoordinates`) as an argument and need no ECM, so they are unit tested on their own (`test_geo`, `test_frame_change`); the ECM wrappers look the world's origin up and call them.

The world's x axis points East. The projection uses the latitude and longitude of `<spherical_coordinates>` only: `<heading_deg>` is not supported and `<elevation>` moves nothing, as z is the altitude from sea level. The MAS logs a warning at load when a world sets either.

Geographic positions have their own types in `geo_types.hpp` (header-only, no gz-sim): `GeoPoint {latitude, longitude, altitude}` in WGS84 degrees and metres from sea level, and `GeoPose {position, orientation}` with the attitude in the ENU world frame, as `geographic_msgs/GeoPose`. A latitude can then never be read as an x. Local positions and attitudes stay `gz::math::Vector3d` and `gz::math::Quaterniond`.

| Function | Description |
|---|---|
| `geoToLocal(origin, GeoPoint)` / `localToGeo(origin, xyz)` → `GeoPoint` | The world's map projection and its exact inverse: x and y are the projection of latitude and longitude onto the plane tangent at the origin, z is the altitude. Gazebo's own conversion gives the 3D point on that plane, whose z drops with the Earth's curvature (about 785 m at 100 km); the projection keeps altitude 0 at the surface. Local scale error about d²/2R² (1e-4 at 100 km) |
| `geoToPose(origin, GeoPoint, heading)` | The level ENU pose of a vessel at a geographic position with a heading |
| `localToGeoPose(origin, pose)` → `GeoPose` | `localToGeo` of the position; the ENU attitude is kept as is |
| `headingToYaw(deg)` / `yawToHeading(rad)` | Heading (degrees from true North, clockwise) to and from ENU yaw |
| `courseOf(east, north)` | Course over ground of a velocity, degrees from true North, clockwise; 360 below 0.05 m/s, as AIS does |
| `wrapDegrees(deg)` / `toFloatDegrees(deg)` | An angle brought into [0, 360), as a double or as a float32 |
| `degToRad` / `radToDeg` | Angle units |
| `quatChangeFrame`, `poseChangeFrame`, `vecBodyChangeFrame`, `pseudoVecBodyChangeFrame`, `worldVelToTargetBody`, `targetBodyVelToWorld` | Changes between axis conventions (ENU/FLU, NED/FRD, EUN/FUL, NEU/FRU), with the `kWorld*` / `kBody*` matrices |
| `worldOrigin(ecm)` | The world's `<spherical_coordinates>`, or `std::nullopt` |
| `worldGeoPose(entity, ecm)` → `std::optional<GeoPose>` | `localToGeoPose` of the entity's world pose with the world's origin; `std::nullopt` if the world has no spherical coordinates |
| `LatLongToXY(ecm, GeoPoint)` → `std::optional<Vector3d>` / `XYToLatLong(ecm, [worldEntity,] xyz)` → `std::optional<GeoPoint>` | `geoToLocal` / `localToGeo` with the world's origin; `std::nullopt` if the world has no spherical coordinates |

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