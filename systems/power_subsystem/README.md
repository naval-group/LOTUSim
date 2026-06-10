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
│              ┌───────────▼──────────────────┐               │
│              │  PlatformPowerManagerBase    │ (one/vessel)  │
│              │  ◄── DefaultPlatformPower..  │               │
│              │                              │               │
│              │  m_providers                 │               │
│              │    ├── Battery*              │               │
│              │    └── Generator*            │               │
│              │  m_consumers                 │               │
│              │    ├── SensorPowerConsumer   │               │
│              │    └── ThrusterPowerConsumer │               │
│              └──────────────────────────────┘               │
└─────────────────────────────────────────────────────────────┘
```

Each vessel spawned that contains a `<lotusim_power>` block in their SDF gets its own `PlatformPowerManagerBase` instance, isolated by vessel name and ROS 2 node namespace. Vessels without `<lotusim_power>` are ignored by this plugin.

---

## Class Hierarchy

```
PowerManager
        |
    PlatformPowerManagerBase  (abstract)
            │   ├── DefaultPlatformPowerManager
            │   └── users implementation
            │
            ├── PowerProvider  (abstract)
            |        ├── Battery       (abstract)
            |        │     └── SimpleBattery
            |        │     └── users implementation
            |        └── Generator     (abstract)
            |            ├── SimpleGenerator
            |            ├── RpmGenerator
            |            └── users implementation
            |
            └── PowerConsumer  (abstract)
                     ├── SensorPowerConsumer
                     ├── ThrusterPowerConsumer
                     └── users implementation
```

---

## Components

### `PowerManager` : World Plugin

The top-level Gazebo system plugin. Declared once in `lotusim.world`.

- Runs `EachNew<Model>` every tick to detect newly spawned vessels
- Checks that the vessel SDF contains at least one `<lotusim_power>` link before creating an instance
- Creates one `PlatformPowerManagerBase` per vessel via `PlatformPowerManagerBase::create()`
- Forwards `Update()` and `PostUpdate()` to all registered instances

**World SDF declaration:**

```xml
<plugin filename="power_subsystem"
        name="lotusim::gazebo::PowerManager">
</plugin>
```

---

### `PlatformPowerManagerBase` : Per-Vessel Manager (abstract)

Owns all power providers and consumers for one vessel. Created by `PowerManager` when a vessel with a valid power config is spawned.

**Responsibilities each tick (`Update`):**

1. Guard: return early if no batteries are registered
2. Compute `dt` from `UpdateInfo`
3. Delegate to `handlePowerUpdate()` — implemented by subclass

**Provider parsing** (`initPowerProvider`) runs at construction: battery and generator SDF is available from `ModelSdf`.

**Consumer parsing** (`initPowerConsumers`) is deferred to the first `PostUpdate` tick — Gazebo registers `CustomSensor` components after the model is constructed, so sensors are not yet in the ECM at construction time.

#### Implementing a custom power manager

Subclass `PlatformPowerManagerBase` and override `handlePowerUpdate`:

```cpp
class MyPowerManager : public PlatformPowerManagerBase {
    void handlePowerUpdate(
        float dt,
        gz::sim::EntityComponentManager& ecm,
        const gz::sim::UpdateInfo& info) final;
};
```

All providers (`m_batteries`, `m_generators`) and consumers (`m_consumers`) are available as protected members. Use `activeBusVoltage()` to read the current bus voltage reference.

---

### `DefaultPlatformPowerManager`

The built-in power manager. Implements the default load distribution, battery switching, and load shedding logic directly in `handlePowerUpdate`.

**`distributeLoad()` — normal mode (batteries available):**

- Battery covers all consumer demand
- Generator charges the active battery if:
  - there is more than one generator available, OR
  - batteries are still active (generator is not the sole power source)
- Charge current is capped at 10 % of the generator's rated output
- Generator does not charge a depleted battery

**`distributeLoad()` — emergency mode (all batteries depleted):**

- Generator covers consumer demand directly
- `reactivateIfPossible()` tries to restore one shed consumer per tick if headroom allows

**`handleDepleted()`:**

- Searches forward from `m_active_battery_index` for the next non-depleted battery
- If found: advances `m_active_battery_index`, returns `true`
- If none found: sets `m_all_batteries_depleted = true`, generator takes over
- If generator cannot cover all consumers, sheds by priority group (4 → 3 → 2)

**`shedLoads()`:**

- Only called when `PowerLevel` is `WARN` or `CRITICAL`
- Skips shedding if another battery or a generator is still available — shedding only fires on the **last available provider**
- Sheds one consumer per tick, highest priority group first (4 → 3 → 2)
- Priority-1 consumers are never shed
- Logs a warning when only safety-critical consumers remain

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

- `PowerLevel` computed from state of charge (SOC) thresholds
- `receiveCharge()` support for generator surplus
- SOC tracking (`initial_soc` from SDF)

**PowerLevel thresholds:**

| Level | Condition |
|---|---|
| `NORMAL` | SOC > 0.20 |
| `WARN` | 0.10 < SOC ≤ 0.20 |
| `CRITICAL` | 0.0 < SOC ≤ 0.10 |
| `DEPLETED` | SOC ≤ 0.001 |

---

### `Generator` : Abstract Base (generators)

Extends `PowerProvider` with:

- `PowerLevel` computed from fuel ratio thresholds
- `surplusChargingCurrent(busCurrentA)` computes available surplus for battery charging
- Fuel tracking via `m_fuel_level` and `m_fuel_capacity`

**PowerLevel thresholds:**

| Level | Condition |
|---|---|
| `NORMAL` | fuel > 20% |
| `WARN` | 10% < fuel ≤ 20% |
| `CRITICAL` | 1% < fuel ≤ 10% |
| `DEPLETED` | fuel ≤ 1% |

---

### `SimpleBattery`

Minimal battery implementation for testing. Models linear voltage drain:

```
remaining_ah -= currentA × dt
voltage = voltage_min + SOC × (voltage_nominal - voltage_min)
```

**SDF:**

```xml
<lotusim_power>
    <name>main_battery</name>
    <type>simple_battery</type>
    <capacity_ah>100</capacity_ah>
    <initial_soc>1.0</initial_soc>
    <voltage_min>36.0</voltage_min>
    <voltage_nominal>48.0</voltage_nominal>
</lotusim_power>
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
<lotusim_power>
    <name>main_generator</name>
    <type>simple_generator</type>
    <fuel_type>diesel</fuel_type>
    <fuel_capacity>500</fuel_capacity>
    <fuel_level_start>400</fuel_level_start>
    <rated_output_w>5000</rated_output_w>
    <efficiency>0.35</efficiency>
    <voltage_nominal>48.0</voltage_nominal>
</lotusim_power>
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
<lotusim_power>
    <name>main_generator</name>
    <type>rpm_generator</type>
    <fuel_type>diesel</fuel_type>
    <fuel_capacity>500</fuel_capacity>
    <fuel_level_start>400</fuel_level_start>
    <rated_output_w>5000</rated_output_w>
    <rated_rpm>2000</rated_rpm>
    <efficiency>0.35</efficiency>
    <voltage_nominal>48.0</voltage_nominal>
</lotusim_power>
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
When deactivated, calls the sensor's `change_state` ROS 2 service
(`/<vessel_name>/<sensor_name>/change_state`) to stop the sensor from publishing.
The service is registered by `CustomSensor::Load()` in `lotusim_sensor_base`.

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

Each tick, `DefaultPlatformPowerManager::handlePowerUpdate` runs the following sequence:

```
1. Sum demand from all active consumers
2. distributeLoad():
       if batteries available:
           battery covers consumer demand
           if spare generator available: generator charges battery
       if all batteries depleted:
           generator covers consumer demand directly
           if 2+ generators: spare generator may charge batteries
3. handleDepleted() (only if active battery is DEPLETED):
       search for next non-depleted battery → switch if found
       if none: generator takes over, shed consumers if needed
4. shedLoads() (only if on last available provider):
       WARN     → shed priority 4 consumers, one per tick
       CRITICAL → shed priority 3 and below, one per tick
       never shed priority 1
5. Push updated voltage to all consumers
6. Publish power status
```

Shedding is skipped as long as another battery or generator is available — the system switches providers first and only sheds when there is truly no alternative.

---

## Adding a New Provider Type

1. Create a subclass of `Battery` or `Generator`
2. Implement `receiveLoad()`, `voltage()`, and any other overrides
3. Add a new `else if (type == "your_type")` branch in `PlatformPowerManagerBase::initPowerProvider()`
4. Add the new `.cpp` to `CMakeLists.txt`

---

## Adding a New Consumer Type

1. Create a subclass of `PowerConsumer`
2. Implement `drawnCurrent()`, `receiveVoltage()`, `isActive()`, `deactivate()`
3. Add a new `else if (powerType == "your_type")` branch in `PlatformPowerManagerBase::initPowerConsumers()`
4. Add the `<lotusim_power>` block to the relevant SDF element
