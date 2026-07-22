# Sensors

The `sensors` subsystem provides custom Gazebo sensors for LOTUSim vessels:  AIS, IMU, radar, and subsea pressure. All of them share a common base class (`lotusim_sensor_base`) and are instantiated by a single world plugin (`lotusim_sensor_plugin`), which handles SDF loading, per-tick updates, rate limiting, and runtime on/off control so individual sensors only need to implement their own measurement logic.

---

## Architecture Overview

```
┌───────────────────────────────────────────────────────────────────┐
│           LotusimSensorPlugin : Gazebo World Plugin               │
│           (one instance for the entire simulation world)          │
│                                                                   │
│  PostUpdate                                                       │
│    EachNew<CustomSensor>     ──► dispatch on gz:type              │
│                                    ──► CreateSensor<T>()          │
│    EachRemoved<CustomSensor> ──► drop from m_entity_sensor_map    │
│                                                                   │
│    for each tracked sensor (if not paused):                       │
│      check PowerStateRegistry ──► skip if powered off             │
│      refresh Position / Orientation / LatLong from ECM            │
│      ──► sensor->UpdateSensor()                                   │
│                                                                   │
│              ┌────────────────────────────────┐                   │
│              │         CustomSensor           │ (one per <sensor>)│
│              │   ◄── AISSensor                │                   │
│              │   ◄── IMUSensor                │                   │
│              │   ◄── RadarSensor              │                   │
│              │   ◄── SubseaPressureSensor     │                   │
│              │   ◄── users implementation     │                   │
│              └────────────────────────────────┘                   │
└───────────────────────────────────────────────────────────────────┘
```

Each `<sensor type="custom" gz:type="...">` element found in a vessel's SDF becomes one `CustomSensor` instance, owned by `LotusimSensorPlugin` for the lifetime of that sensor entity.

---

## Class Hierarchy

```
LotusimSensorPlugin  (Gazebo world System plugin)
        │
        └── owns one CustomSensor per <sensor> entity
                    │
                    CustomSensor  (abstract, extends gz::sensors::Sensor)
                        ├── AISSensor
                        ├── IMUSensor
                        ├── RadarSensor
                        ├── SubseaPressureSensor
                        └── users implementation
```

---

## Components

### `LotusimSensorPlugin` : World Plugin

The top-level Gazebo system plugin - declared once in the world SDF.

```xml
<plugin filename="lotusim_sensor_plugin" name="lotusim::sensor::LotusimSensorPlugin">
</plugin>
```

- **`Configure()`** - creates the plugin's ROS 2 node and logger, and subscribes to Gazebo's `/collision` topic to aggregate contacts for the `collisions` output topic.
- **`PostUpdate()`**
  - `EachNew<CustomSensor, ParentEntity>` → internal `EachNew()` reads the sensor's `gz:type` and instantiates the matching subclass via `CreateSensor<T>()`, then stores it in `m_entity_sensor_map`.
  - `EachRemoved<CustomSensor>` → drops the corresponding entry from `m_entity_sensor_map`.
  - For every tracked sensor, if the simulation isn't paused:
    1. Looks up the sensor's power state in `PowerStateRegistry` (keyed `<vessel>/<sensor>`) - skips the update if the power subsystem has switched it off. Defaults to `true` (always on) if no power manager is present.
    2. Refreshes the sensor's world position, orientation, and lat/long from the ECM.
    3. Calls `sensor->UpdateSensor(info, ecm)`.
  - Publishes aggregated collisions on `collisions` (`lotusim_sensor_msgs/msg/Collisions`).

---

### `CustomSensor` : Abstract Base (`lotusim_sensor_base`)

Every sensor type derives from `CustomSensor`, itself a `gz::sensors::Sensor`. It provides the shared plumbing so each concrete sensor only implements its own physics and message.

**Provided for free by the base class:**

| Feature | Description |
|---|---|
| `update_rate` / `is_on` SDF parsing | Parsed once in `Load()` |
| `EnableMeasurement(now)` | Rate-limits updates to `update_rate` Hz and respects the on/off state - call this as a guard at the top of `UpdateSensor()` |
| `<vessel>/<sensor>/change_state` ROS 2 service | Lets external code turn the sensor on/off at runtime |
| `noise_sigma` / `noise_amplitude` SDF parsing + `GetGaussianNoise()` | Ready-to-use Gaussian noise helper |
| `Position()` / `LatLong()` / `Orientation()` | Called every tick by the plugin, so `m_position` / `m_lat_long` / `m_quad` are always current |

**What each subclass must implement:**

| Method | Purpose |
|---|---|
| `CustomSensorLoad(sdf)` | Parse sensor-specific SDF parameters, create the ROS 2 publisher(s) |
| `UpdateSensor(info, ecm)` | Called at up to `update_rate` Hz - compute and publish a measurement |

---

## Existing Sensors

| Sensor | `gz:type` | Publishes | Notes |
|---|---|---|---|
| AIS | `ais` | `<vessel>/<sensor>/ais` (`lotusim_sensor_msgs/msg/AIS`) | Speed-over-ground and heading, derived from the `base_link`'s world velocity |
| IMU | `imu` | `<vessel>/<sensor>/IMU` (`sensor_msgs/msg/Imu`) | Orientation, angular velocity, linear acceleration |
| Radar | `radar` | see [Radar Sensor README](radar_sensor/README.md) | Needs a co-located `gpu_lidar` sensor on the same link |
| Subsea Pressure | `subsea_pressure` | `<vessel>/<sensor>/pressure_sensor` (`lotusim_sensor_msgs/msg/PressureDepth`) | Pressure derived from depth; can optionally back-estimate depth from pressure |

Minimal SDF for AIS and IMU:

```xml
<sensor name="ais_sensor" type="custom" gz:type="ais">
    <update_rate>1</update_rate>
    <noise_sigma>0.01</noise_sigma>
    <noise_amplitude>0.01</noise_amplitude>
</sensor>

<sensor name="imu_sensor" type="custom" gz:type="imu">
    <update_rate>1</update_rate>
    <noise_sigma>0.01</noise_sigma>
    <noise_amplitude>0.01</noise_amplitude>
</sensor>
```

Subsea pressure adds its own parameters:

```xml
<sensor name="pressure_sensor" type="custom" gz:type="subsea_pressure">
    <update_rate>1</update_rate>
    <m_saturation>3000</m_saturation>          <!-- kPa, output cap -->
    <estimate_depth_on>true</estimate_depth_on>
    <standard_pressure>101.325</standard_pressure>  <!-- kPa at surface -->
    <kPa_per_meter>9.80638</kPa_per_meter>
</sensor>
```

For radar's setup (it needs a paired LiDAR sensor and has several tuning parameters), see the [Radar Sensor README](radar_sensor/README.md).

Every sensor also accepts an optional `<lotusim_power>` block to draw from the vessel's battery - see the [Power Subsystem README](../../power_subsystem/README.md#add-a-sensor).

---

## Adding a New Sensor

A new sensor is a new ROS 2 package under `systems/sensors/` that subclasses `CustomSensor`, plus a small amount of wiring in `lotusim_sensor_plugin` so the plugin knows how to instantiate it. `imu_sensor` is the simplest existing example to copy from.

**1. Create the package**

```
systems/sensors/my_sensor/
├── CMakeLists.txt
├── package.xml
├── include/my_sensor/my_sensor.hpp
└── src/my_sensor.cpp
```

Model `CMakeLists.txt` on `imu_sensor/CMakeLists.txt`: it needs `find_package(lotusim_sensor_base REQUIRED)`, `find_package(lotusim_common REQUIRED)`, plus whatever message package you publish, and builds a single shared library from your `.cpp` file(s).

**2. Subclass `CustomSensor`**

```cpp
// my_sensor.hpp
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "my_msgs/msg/my_reading.hpp"

namespace lotusim::sensor {

class MySensor : public CustomSensor {
public:
    MySensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    ~MySensor();

    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor& _sdf) final;

    rclcpp::Publisher<my_msgs::msg::MyReading>::SharedPtr m_sensor_pub;
};

}  // namespace lotusim::sensor
```

**3. Implement `CustomSensorLoad()`** - parse any sensor-specific SDF parameters with `GetSDFParam<T>()` and create your publisher. Keep the `<vessel>/<sensor>/<topic>` naming convention used by every existing sensor:

```cpp
bool MySensor::CustomSensorLoad(const sdf::Sensor& _sdf)
{
    sdf::ElementPtr sdfptr = _sdf.Element();
    GetSDFParam<double>(sdfptr, "my_param", m_my_param, /*default=*/1.0);

    m_sensor_pub = m_ros_node->create_publisher<my_msgs::msg::MyReading>(
        m_vessel_name + "/" + m_sensor_name + "/my_reading",
        rclcpp::QoS(1));
    return true;
}
```

**4. Implement `UpdateSensor()`** - guard with `EnableMeasurement()`, read whatever you need from the ECM (or the base class's `m_position` / `m_lat_long` / `m_quad`, already refreshed for you every tick), publish, and stamp `m_last_measurement_time`:

```cpp
bool MySensor::UpdateSensor(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager&)
{
    if (!EnableMeasurement(_info.simTime))
        return false;

    my_msgs::msg::MyReading msg;
    msg.header = lotusim::common::generateHeaderMessage(_info.simTime);
    msg.value = m_my_param + GetGaussianNoise(m_noise_amp);

    m_sensor_pub->publish(msg);
    m_last_measurement_time = _info.simTime;
    return true;
}
```

**5. Wire it into `lotusim_sensor_plugin`**:

- Add `#include "my_sensor/my_sensor.hpp"` to `lotusim_sensor_plugin.hpp`.
- Add a branch in `LotusimSensorPlugin::EachNew()` (`lotusim_sensor_plugin.cpp`):
  ```cpp
  } else if (type == "my_sensor") {
      sensor = CreateSensor<MySensor>(
          data, model_entity, _entity, model_name, sensor_name);
  }
  ```
- In `lotusim_sensor_plugin/CMakeLists.txt`, add `find_package(my_sensor REQUIRED)` and list `my_sensor` in both `ament_target_dependencies()` and `ament_export_dependencies()`.

> **Note:** sensor type resolution is a hardcoded `if / else if` chain on the `gz:type` string, not a runtime plugin registry. Every new sensor type means editing and rebuilding `lotusim_sensor_plugin` itself - it's not something you can drop in as an independent, self-registering package.

**6. Declare it on a vessel**

```xml
<sensor name="my_sensor_1" type="custom" gz:type="my_sensor">
    <update_rate>1</update_rate>
    <my_param>2.5</my_param>
</sensor>
```

The `gz:type` value must exactly match the string compared in step 5.

**7. Rebuild and relaunch** - `lotusim clean_build` (or `colcon build`, depending on which workspace you're in), then re-run your scenario.