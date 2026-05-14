# Power Subsystem

The `power_subsystem` is a Gazebo world plugin that models the electrical power architecture. It tracks battery state, generator fuel consumption, consumer power draw, and automatically sheds non-critical loads when power is running low.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│              PowerManager : Gazebo World Plugin             │
│       (one instance for the entire simulation world)        │
│                                                             │
│   EachNew<Model> ──► loadVessel()                           │
│                          │                                  │
│              ┌───────────▼────────────┐                     │
│              │  PowerManagerInstance  │  (one per vessel)   │
│              │                        │                     │
│              │  m_providers           │                     │
│              │    ├── Battery*        │                     │
│              │    └── Generator*      │                     │
│              │  m_consumers           │                     │
│              │    ├── SensorPower...  │                     │
│              │    └── ThrusterPower.. │                     │
│              └────────────────────────┘                     │
└─────────────────────────────────────────────────────────────┘
```

Each vessel spawned that contains `<lotusim_power>` block in their SDF gets its own `PowerManagerInstance`, isolated by vessel name and ROS 2 node namespace. Vessels without `<lotusim_power>` are ignored bu this plugin.

---

## Class Hierarchy

```
PowerManager
        |
    PowerManagerInstance
            ├── PowerProvider  (abstract)
            |        ├── Battery       (abstract)
            |        │     └── SimpleBattery
            |        └── Generator     (abstract)
            |            ├── SimpleGenerator
            |            └── RpmGenerator
            |
            ├── PowerConsumer  (abstract)
                    ├── SensorPowerConsumer
                    └── ThrusterPowerConsumer
```

---

## Components

### `PowerManager` : World Plugin

The top-level Gazebo system plugin. Declared once in `lotusim.world`.

- Runs `EachNew<Model>` every tick to detect newly spawned vessels
- Checks that the vessel SDF contains at least one `<lotusim_power>` link before creating an instance
- Creates one `PowerManagerInstance` per vessel, each with its own namespaced ROS 2 node (`vessel_name/power_subsystem`)
- Forwards `Update()` and `PostUpdate()` to all registered instances

**World SDF declaration:**

```xml
<plugin filename="power_subsystem"
        name="lotusim::gazebo::PowerManager">
</plugin>
```

---

### `PowerManagerInstance` : Per-Vessel Manager

Owns all power providers and consumers for one vessel. Created by `PowerManager` when a vessel with a valid power config is spawned.

**Responsibilities each tick (`Update`):**

1. Detect sensor topology changes (`EachNew`, `EachRemoved`)
2. Sum total current demand from all active consumers
3. Distribute load: generator first, battery covers the shortfall
4. If generator has surplus energy -> charge battery
5. Check battery `PowerLevel` and switch to next battery if depleted
6. Shed non-critical loads based on `PowerLevel`
7. Push updated voltage to all consumers

**Provider parsing** (`parsePowerProviders`) runs at construction : battery and generator SDF is available from `ModelSdf`.

**Consumer parsing** (`parsePowerConsumers`) is deferred to the first `PostUpdate` tick -> Gazebo registers `CustomSensor` components after the model is constructed, so sensors are not yet in the ECM at construction time.

---

### `PowerProvider` : Abstract Base (providers)

Defines the contract for all power sources:

| Method | Description |
|---|---|
| `receiveLoad(currentA, dt)` | Accept current draw, update internal state |
| `voltage()` | Current output voltage (V) |
| `getStateOfCharge()` | Normalised remaining capacity (0–1) |
| `availablePowerW()` | Estimated sustainable power output (W) |
| `powerLevel()` | Health level: `NORMAL / WARN / CRITICAL / DEPLETED` |
| `isDepleted()` | True when provider can no longer supply power |
| `canReceiveCharge()` | True for batteries, false for generators |
| `receiveCharge(currentA, dt)` | Accept surplus charging current (batteries only) |
| `name()` | Provider name from SDF |

---

### `Battery` : Abstract Base (batteries)

Extends `PowerProvider` with:

- `PowerLevel` computed from voltage vs `voltage_min` thresholds
- `receiveCharge()` support for generator surplus
- SOC tracking (`initial_soc` from SDF)

**PowerLevel thresholds:**

| Level | Condition |
|---|---|
| `NORMAL` | `voltage > voltage_min × 1.15` |
| `WARN` | `voltage_min × 1.05 < voltage ≤ voltage_min × 1.15` |
| `CRITICAL` | `voltage_min < voltage ≤ voltage_min × 1.05` |
| `DEPLETED` | `voltage ≤ voltage_min` |

---

### `Generator` : Abstract Base (generators)

Extends `PowerProvider` with:

- `PowerLevel` computed from fuel ratio thresholds
- `surplusChargingCurrent(busCurrentA)` computes available surplus for battery charging
- Fuel tracking via `m_fuel_level` and `m_fuel_capacity`

**PowerLevel thresholds:**

| Level | Condition |
|---|---|
| `NORMAL` | fuel > 25% |
| `WARN` | 10% < fuel ≤ 25% |
| `CRITICAL` | 5% < fuel ≤ 10% |
| `DEPLETED` | fuel ≤ 5% |

---

### `SimpleBattery`

Minimal battery implementation for testing:

```
voltage -= (currentA × dt) / capacity_ah
```

**SDF:**

```xml
<link name="main_battery">
    <lotusim_power>
        <type>simple_battery</type>
        <capacity_ah>100</capacity_ah>
        <initial_soc>1.0</initial_soc>
        <voltage_min>36.0</voltage_min>
        <voltage_nominal>48.0</voltage_nominal>
    </lotusim_power>
</link>
```

| Parameter | Unit | Description |
|---|---|---|
| `capacity_ah` | Ah | Battery capacity |
| `initial_soc` | 0–1 | Starting state of charge |
| `voltage_min` | V | Depletion voltage -> PowerLevel thresholds derived from this |
| `voltage_nominal` | V | Full-charge voltage |

---

### `SimpleGenerator`

Minimal generator for testing, models linear fuel consumption:

```
power_w         = currentA × voltage_nominal
fuel_consumed_l = (power_w × dt) / (efficiency × energy_density_J_per_L)
```

**SDF:**

```xml
<link name="main_generator">
    <lotusim_power>
        <type>simple_generator</type>
        <fuel_type>diesel</fuel_type>
        <fuel_capacity>500</fuel_capacity>
        <fuel_level_start>400</fuel_level_start>
        <rated_output_w>5000</rated_output_w>
        <efficiency>0.35</efficiency>
        <voltage_nominal>48.0</voltage_nominal>
    </lotusim_power>
</link>
```

| Parameter | Unit | Description |
|---|---|---|
| `fuel_type` | string | See `fuel_properties.hpp` for supported types |
| `fuel_capacity` | L | Full tank size |
| `fuel_level_start` | L | Fuel at simulation start |
| `rated_output_w` | W | Maximum continuous output |
| `efficiency` | 0–1 | Fuel-to-electrical conversion ratio |
| `voltage_nominal` | V | Stable output voltage |

---

### `RpmGenerator`

Physics-based generator where fuel consumption is driven by a live RPM topic. Uses the **propeller cubic law**:

```
power_w         = rated_output_w × (rpm / rated_rpm)³
fuel_consumed_l = (power_w × dt) / (efficiency × energy_density_J_per_L)
```

Subscribes to `/<vessel_name>/rpm` (`std_msgs/Float64`). At RPM = 0, no fuel is consumed and no power is produced.

**SDF:**

```xml
<link name="main_generator">
    <lotusim_power>
        <type>rpm_generator</type>
        <fuel_type>diesel</fuel_type>
        <fuel_capacity>500</fuel_capacity>
        <fuel_level_start>400</fuel_level_start>
        <rated_output_w>5000</rated_output_w>
        <rated_rpm>2000</rated_rpm>
        <efficiency>0.35</efficiency>
        <voltage_nominal>48.0</voltage_nominal>
    </lotusim_power>
</link>
```

| Parameter | Unit | Description |
|---|---|---|
| `rated_rpm` | RPM | RPM at which generator produces `rated_output_w` |

**RPM topic publisher** -> the scenario script must publish RPM on `/<vessel_name>/rpm`:

```python
from std_msgs.msg import Float64

self.rpm_publishers[vessel_name] = self.create_publisher(
    Float64, f'/{vessel_name}/rpm', 10)

rpm_msg = Float64()
rpm_msg.data = float(rpm)
self.rpm_publishers[vessel_name].publish(rpm_msg)
```

---

### `SensorPowerConsumer`

Represents a sensor declared in the vessel SDF. Draws a fixed rated power when active.

**SDF** add a `<lotusim_power>` block inside the `<sensor>` element:

```xml
<sensor name="ais" type="custom" gz:type="ais">
    <lotusim_power>
        <power_type>sensor</power_type>
        <nominal_w>5.0</nominal_w>
        <priority>3</priority>
    </lotusim_power>
    <update_rate>1</update_rate>
</sensor>
```

---

### `ThrusterPowerConsumer`

Represents a thruster declared in the vessel SDF. Add a `<thruster>` element inside a `<link>`:

```xml
<link name="thruster_link">
    <thruster name="main_thruster">
        <nominal_w>1500</nominal_w>
        <priority>2</priority>
    </thruster>
</link>
```

---

### `fuel_properties.hpp`

Lookup table mapping fuel type strings to energy density in J/L. Used by `SimpleGenerator` and `RpmGenerator` to compute fuel consumption.

**Supported fuel types:**

| Fuel | Energy density | Notes |
|---|---|---|
| `diesel` | 34,920,000 J/L | ~9.7 kWh/L |
| `gasoline` | 32,400,000 J/L | ~9.0 kWh/L |
| `lng` | 22,320,000 J/L | Liquefied natural gas |
| `hydrogen` | 9,000,000 J/L | Compressed at 700 bar |
| `methanol` | 15,840,000 J/L | ~4.4 kWh/L |

To add a new fuel type, add one entry to `kFuelEnergyDensityJPerL` in `fuel_properties.hpp` -> no other code changes required. An unrecognised `fuel_type` in the SDF throws `std::invalid_argument` at simulation start.

---

## Priority Levels

Load shedding is priority-based. Lower number = higher priority = shed last.

| Priority | Category | Examples | Shed when |
|---|---|---|---|
| 1 | Safety critical | Navigation, emergency lighting | Never |
| 2 | Operationally important | Thrusters | Last resort only |
| 3 | Mission systems | AIS, sonar | `CRITICAL` level |
| 4 | Non-essential | Deck lighting, comfort systems | `WARN` level |

---

## Load Distribution Logic

Each tick, `PowerManagerInstance` distributes load in this order:

```
1. Find first non-depleted generator (SDF order)
2. If generator output ≥ demand:
       generator runs at demand level
       surplus → charge active battery
       if active battery full → charge any non-full battery
       if all batteries full → surplus is lost
3. If generator output < demand:
       generator runs at full capacity
       battery covers the shortfall
4. If no generators, or all depleted:
       battery covers full demand
5. If all batteries depleted:
       all consumers deactivated
```

Multiple generators are prioritised by SDF declaration order, the first non-depleted generator always takes the full load before the next one is considered.

---

## Adding a New Provider Type

1. Create a subclass of `Battery` or `Generator`
2. Implement `receiveLoad()`, `voltage()`, and any other overrides
3. Add a new `else if (type == "your_type")` branch in `PowerManagerInstance::parsePowerProviders()`
4. Add the new `.cpp` to `CMakeLists.txt`

---

## Adding a New Consumer Type

1. Create a subclass of `PowerConsumer`
2. Implement `drawnCurrent()`, `receiveVoltage()`, `isActive()`, `deactivate()`
3. Add a new `else if (powerType == "your_type")` branch in `PowerManagerInstance::parsePowerConsumers()`
4. Add the `<lotusim_power>` block to the relevant SDF element