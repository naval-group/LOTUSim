# Doppler Velocity Log (DVL) Sensor

A custom LOTUSim sensor (`gz:type="dvl"`) that reports vessel velocity along 4 acoustic beams in a Janus configuration. Currently, this is only working if you spawn a vessel with dynamism (using xdyn), not supported when using the waypoint follower.

## How it works

The sensor has 2 operational modes, depending on whether a seafloor heightmap is configured or not. Both modes run every tick, the sensor doesn't switch between them, it simply reports `bottom_lock: false` and
placeholder ranges whenever bottom data isn't available or isn't valid.

### 1. Water-track mode (always active)

This is the default behaviour and requires no extra configuration:

- The sensor reads the vessel's true world-frame linear velocity directly from the physics engine (`gz::sim::Link::WorldLinearVelocity`).
- That velocity is rotated into the vessel's body frame, then projected onto each of the 4 beam directions (`velocity · beam_direction`).
- Per-axis Gaussian noise (configured via the standard `<noise>` SDF block) is applied to both the body-frame velocity and each beam's projected velocity before publishing.

**Important limitation:** Water currents is not taken into account currently. "Velocity relative to the world frame" is used as a stand-in for "velocity relative to water," which is only accurate when the water is static. If a current/flow field is added to LOTUSim later, the local current vector at the vessel's position should be subtracted from the world velocity before projecting onto beams.

### 2. Bottom-track mode (optional, requires `<lotusim_seafloor>`)

If `<lotusim_seafloor>` is present in the sensor's SDF, the sensor:

- Loads a grayscale PNG heightmap once, at startup (`CustomSensorLoad`).
- Every tick, samples the heightmap at the vessel's current `(x, y)` world position to estimate seafloor elevation (`z`) at that point.
- Computes `altitude = vessel_z - seafloor_z`.
- Sets `bottom_lock = true` only if `0 < altitude <= max_altitude` (e.g the vessel is above the seafloor and within the sensor's realistic detection range).
- When `bottom_lock` is true, estimates a per-beam range to the bottom using a **locally-flat-seafloor approximation**: 
     `beam_range = altitude / cos(beam_angle)`.
  This is a simplification, not true raycasting along the tilted beam direction against the actual (possibly sloped) terrain under each beam: accurate for gently sloping seafloor, less accurate near steep features.
- When `bottom_lock` is false (no seafloor configured, vessel too high, or outside the mapped heightmap extent), `beam_range` is set to `-1.0` for all beams to signal "no valid range."

Heightmap pixel brightness maps linearly to world-frame elevation:
darkest pixel → `min_depth`, brightest pixel → `max_depth`. Sampling is nearest-pixel (no interpolation).

## SDF configuration

```xml
<sensor name="dvl_sensor" type="custom" gz:type="dvl">
    <!-- Optional: integrates with LOTUSim's power simulation, same as
         other sensors. 
    <lotusim_power>
        <type>sensor</type>
        <nominal_w>3.0</nominal_w>
        <priority>3</priority>
    </lotusim_power>
    -->

    <always_on>true</always_on>
    <update_rate>5</update_rate>

    <!-- Beam geometry -->
    <beam_angle_deg>30</beam_angle_deg>

    <!-- Standard noise block -->
    <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
    </noise>

    <!-- Optional: enables bottom-track mode. Omit this whole block to
         run in water-track-only mode. -->
    <lotusim_seafloor>
        <png_path>model://seabed/seabed.png</png_path>
        <size_x>1000</size_x>       <!-- world extent covered by the image, metres -->
        <size_y>1000</size_y>
        <origin_x>0</origin_x>      <!-- world (x, y) of the image's mapped origin -->
        <origin_y>0</origin_y>
        <min_depth>-50</min_depth>  <!-- world z at the darkest pixel (deepest point) -->
        <max_depth>0</max_depth>    <!-- world z at the brightest pixel (shallowest point) -->
    </lotusim_seafloor>

    <max_altitude>1000</max_altitude>  <!-- max height above seafloor for a valid bottom lock -->
</sensor>
```

| Param | Default | Meaning |
|---|---|---|
| `beam_angle_deg` | `30.0` | Half-angle of each beam off the downward vertical (`-Z`, body frame). 4 beams are placed 90° apart in azimuth (Janus configuration). |
| `noise` | none | Standard SDF Gaussian noise block; applied independently to body-frame velocity (x, y, z) and to each beam's projected velocity. |
| `lotusim_seafloor` | absent (water-track only) | See table below. If absent, the sensor runs in water-track mode only: `bottom_lock` is always `false` and `beam_range` is always `-1.0`. |
| `max_altitude` | `150.0` m | Maximum vessel altitude above the seafloor for `bottom_lock` to be `true`. Real DVLs have a hard acoustic range limit; tune this to match the class of device you're modeling (see note below). |

### `<lotusim_seafloor>` sub-params

| Param | Required | Meaning |
|---|---|---|
| `png_path` | yes | Path to a grayscale heightmap image (`model://...` URIs are resolved the same way meshes are). |
| `size_x`, `size_y` | no (default 1000m each) | World-frame extent, in metres, that the image covers. |
| `origin_x`, `origin_y` | no (default 0, 0) | World `(x, y)` coordinate mapped to the image's origin corner. |
| `min_depth`, `max_depth` | no (default -50, 0) | World `z` elevation at the darkest and brightest pixel values, respectively. Linearly interpolated between the two based on pixel brightness. |

### Choosing `max_altitude`

Real DVL range depends heavily on acoustic frequency:

| Device class | Typical max altitude |
|---|---|
| High-frequency (small AUV/ROV, e.g. 1200 kHz) | ~25–30 m |
| Mid-frequency (600 kHz) | ~120–150 m |
| Lower-frequency (300 kHz) | ~150–300 m |
| Long-range / deep-water specialist units | several hundred metres, up to ~1000 m |

## Message output

Published on `<vessel_name>/<sensor_name>/dvl` as `lotusim_msgs::msg::DVL`:

```
std_msgs/Header header
geometry_msgs/Vector3 velocity   # body-frame velocity (m/s), water-track
bool bottom_lock                 # true if within max_altitude of the seafloor
float64[4] beam_velocity         # velocity projected onto each beam (m/s)
float64[4] beam_range            # range to bottom per beam (m), -1.0 if no lock
```

Beam order in `beam_velocity` / `beam_range` follows azimuth `0°, 90°, 180°, 270°` (indices 0–3), matching the order beams are constructed in `CustomSensorLoad`.

## Usage example

Minimal water-track-only DVL (no seafloor needed):

```xml
<sensor name="dvl_sensor" type="custom" gz:type="dvl">
    <always_on>true</always_on>
    <update_rate>5</update_rate>
    <beam_angle_deg>30</beam_angle_deg>
    <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
    </noise>
</sensor>
```

1. Run XDYN

In a first terminal, run:
   ```shell
   xdyn-for-cs $HOME/lotusim_ws/src/LOTUSim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
   ```

2. Run LOTUSim

In a second terminal, run:
   ```shell
   lotusim run
   ```

3. Spawn the ship & send commands

In a third terminal, run:
   ```shell
   python3 controlling_ships.py 
   ```

4. Start the viewer

In a fourth terminal, run:

   ```shell
python3 systems/sensors/doppler_vel_log/scripts/seafloor_viewer.py \
    --seafloor $HOME/lotusim_ws/src/LOTUSim/assets/models/seabed/seabed.png \
    --size-x 1000 --size-y 1000 \
    --vessels lrauv_0
   ```

5. Subscribe from ROS 2:

```bash
ros2 topic echo /lrauv_0/dvl_sensor/dvl
```

## Known limitations

- **No water current simulation.** World-frame velocity stands in for water-relative velocity. Accurate only when water is static.
- **Bottom range is an approximation**, not true per-beam raycasting against the (possibly sloped) heightmap. It assumes a locally flat seafloor directly beneath the vessel.
- **Nearest-pixel heightmap sampling**, not bilinear interpolation, fine for smooth/gradual terrain, coarser near sharp features or at low heightmap resolution.
- **Only meaningful on physics-driven vessels.** The DVL reads velocity from `gz::sim::Link::WorldLinearVelocity()`, which is populated by rigid-body physics integration. Vessels that are moved *kinematically*
  - e.g by directly setting their `Pose` component each tick, rather than being driven by a physics engine do **not** produce meaningful data here. `WaypointFollowerPlugin`, for example, moves vessels this way when no `<physics_engine_interface>` is configured for them. On such vessels, `WorldLinearVelocity()` returns near-zero or stale data, and the DVL's output is dominated entirely by the configured `<noise>` term rather than real motion.