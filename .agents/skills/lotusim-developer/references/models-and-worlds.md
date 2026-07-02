# Anatomy of a model and a world

> **Scope**: this file covers the **model** (assets, in the **core**) and the
> **world**. A vessel's **behavior/controller** does **not** live here — it goes
> in **`LOTUSim-generic-scenario`** (`src/agents/`), see `architecture.md`.

## Model: `assets/models/<name>/`

```
<name>/
├── model.config     # Gazebo metadata, points at model.sdf
├── model.sdf        # SDF (version varies by model, 1.6–1.10): links, collision, sensors (often NO visual)
├── <name>.yaml      # xdyn model: ALL the dynamics (naming is loose: .yaml/.yml/…Config.yaml)
└── meshes/          # .dae (rendering) + .stl (xdyn hydrostatics)
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>BlueBoat</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author><name>...</name></author>
  <maintainer>...</maintainer>
  <description>...</description>
</model>
```

### model.sdf (minimal — by design)

The `model.sdf` is **small**: a `base_link` with a **collision** geometry and
custom sensors. **No physics plugin here** (the dynamics come from xdyn), and
often **no `<visual>` or inertial**. A typical AIS sensor: `gz:type="ais"`.

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <model name="blueboat">
    <link name="base_link">
      <collision name="base_collision">
        <geometry><mesh><uri>model://blueboat/meshes/blueboat.stl</uri></mesh></geometry>
      </collision>
      <!-- ADD to see it in the Gazebo GUI (cf. fremm/commando): -->
      <visual name="visual">
        <geometry><mesh><uri>model://blueboat/meshes/blueboat.dae</uri></mesh></geometry>
      </visual>
      <sensor name="ais" type="custom" gz:type="ais">
        <update_rate>1</update_rate>
        <noise_sigma>0.01</noise_sigma>
        <noise_amplitude>0.01</noise_amplitude>
      </sensor>
    </link>
  </model>
</sdf>
```

### Visual vs Unity (important)

| Rendered in | Models | `<visual>` in model.sdf |
|---|---|---|
| **Unity** (`render_plugin`→ROS2→prefab) | wamv, dtmb_hull, lrauv, mine, pha, bluerov2_heavy | no (collision only) |
| **Gazebo** (direct) | fremm, commando, landscape, x500 | yes |

Consequence: a "collision-only" model is **invisible in the gz GUI** (toggle
*Entity Tree → right-click → View → Collisions* to see it). Giving a vehicle a
Unity look = a **HDRP prefab** in `LOTUSim-Unity-modules` + a name→prefab mapping
(`LotusimBaseInterface.cs`, *"Maps vessel names to prefab names"*) — a separate,
heavy art pipeline. For a self-contained demo/contribution: give it a gz
`<visual>` (a legitimate pattern, cf. fremm/commando).

⚠️ A `<visual>` alone is not enough to SEE it in the gz GUI: many LOTUSim worlds
(e.g. `xdyn_multithread_test`) don't embed the `SceneBroadcaster` (they render in
Unity; some worlds like `defenseScenario` do include it). Add to the world
`<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>`
to render the `<visual>` in gz (and publish the pose topics, cf. the oracle).

⚠️ **Orientation — LOTUSim convention: the mesh's bow is on +y** (measured on
`fremm.dae`: 142 m in y, 24 m in x). A bow-on-+x mesh renders **sideways**
("crabbing", 90° off heading). Fix: model the bow on +y, or add
`<pose>0 0 0 0 0 1.5708</pose>` on the `<visual>`. The physics (STL/xdyn) is not
affected — this is purely rendering.

### The xdyn model file (`.yaml`/`.yml`) — the dynamics

This is where the physics lives. Structure (cf. `wamv.yaml`, `dtmb-xdyn.yml`,
`lrauv.yml`):

```yaml
rotations convention: [psi, theta', phi'']
environmental constants:
    g:   {value: 9.81, unit: m/s^2}
    rho: {value: 1025, unit: kg/m^3}        # seawater
    nu:  {value: 1.18e-6, unit: m^2/s}
environment models:
    - model: no wind
    - model: no waves
      constant sea elevation in NED frame: {value: 0, unit: m}
    - model: ekman current        # or 'no current'
      ...
bodies:
  - name: BLUEBOAT
    mesh: blueboat/meshes/blueboat.stl       # for hydrostatics + Froude-Krylov
    position of body frame relative to mesh: { frame: mesh, x/y/z/phi/theta/psi }
    initial position of body frame relative to NED: { frame: NED, ... }
    initial velocity of body frame relative to NED: { frame: BLUEBOAT, u/v/w/p/q/r }
    dynamics:
        hydrodynamic forces calculation point in body frame: { x,y,z }
        centre of inertia: { frame: BLUEBOAT, x,y,z }
        rigid body inertia matrix at the center of gravity and projected in the body frame:
            row 1..6: [ ... 6x6 ... ]
        added mass matrix at the center of gravity and projected in the body frame:
            row 1..6: [ ... 6x6 ... ]
        # + damping, and PROPULSION (named thrusters — see below)
```

**Propulsion** defines **named** actuators (e.g. `PSPropRudd`, `SBPropRudd` on
dtmb for a twin layout) — these names must match the world's `<thrusters>`. Take
inspiration from `lrauv` (Wageningen B-series propeller) and `dtmb_hull` (twin).
Deprecated xdyn keys are tolerated (warnings "center of buoyancy" → prefer
"center of gravity").

### Generating the meshes (Blender)

Two meshes per model: **`<name>.dae`** (COLLADA visual, if the model gets a
`<visual>`) and **`<name>.stl`** (a *closed* hull for xdyn hydrostatics,
referenced by `<name>.yaml`).

- **Blender 4.5 LTS, not 5.0**: the **COLLADA `.dae`** exporter exists in 4.5
  (removed in 5.0).
- **STL: normals facing outward.** xdyn warns `N facets seem oriented inwards`
  if the normals are inconsistent (a vertex edit can flip them) → wrong
  hydrostatics. In Blender: Edit Mode → `normals_make_consistent(inside=False)`
  before the STL export.
- **Unity import gotcha**: Blender 4.5's COLLADA `.dae` export is **broken for
  Unity import** (produces empty `<instance_geometry>` → empty import). For Unity,
  export **FBX** with `bake_space_transform=True, axis_forward='-Z', axis_up='Y'`
  (Blender's Z-up → Unity's Y-up baked into the vertices, root at identity —
  critical, since the pose flow overwrites the root rotation).

## World: `assets/worlds/*.world`

Plugins **at world level** (once), then each vehicle as an `<include>`.

```xml
<sdf version="1.7">
  <world name="lotusim">
    <gravity>0 0 0</gravity>          <!-- xdyn does ALL the forces, not gz -->
    <plugin filename="physics_interface_plugin" name="lotusim::gazebo::PhysicsInterfacePlugin"/>
    <plugin filename="entity_manager_plugin"   name="lotusim::gazebo::EntityManager"/>
    <plugin filename="lotusim_sensor_plugin"   name="lotusim::sensor::LotusimSensorPlugin"/>
    <plugin filename="render_plugin"           name="lotusim::gazebo::RenderPlugin">
        <connection_protocol>ROS2</connection_protocol>
    </plugin>
    <plugin filename="waypoint_plugin"         name="lotusim::gazebo::WaypointFollowerPlugin"/>

    <include>
      <uri>model://blueboat</uri>
      <name>blueboat1</name>
      <pose>0 0 0 0 0 0</pose>
      <lotus_param>
        <physics_engine_interface>            <!-- wire the vessel to an xdyn server -->
          <surface>
            <connection_type>XDynWebSocket</connection_type>
            <uri>ws://127.0.0.1:12345</uri>   <!-- one unique port per vessel -->
            <thrusters>
              <thruster1>PSPropRudd</thruster1>   <!-- names = those in the xdyn yaml -->
              <thruster2>SBPropRudd</thruster2>
            </thrusters>
          </surface>
          <init_state>Surface</init_state>
        </physics_engine_interface>
        <render_interface>
          <publish_render>true</publish_render>
          <renderer_type_name>wamv</renderer_type_name>   <!-- Unity prefab key -->
        </render_interface>
      </lotus_param>
    </include>
  </world>
</sdf>
```

⚠️ `<physics_engine_interface>` (co-sim) and `<waypoint_follower>` are
**mutually exclusive** per vessel — no shipped world combines them (cf. pitfall
#6). The `<include>` above is the co-sim path; for a purely kinematic vessel,
replace `<physics_engine_interface>` with a `<waypoint_follower>` block (see
`circling_ship_example.world`).

Reference worlds: `circling_ship_example.world` (1 kinematic vessel, no
`physics_engine_interface`), `xdyn_multithread_test.world` (2 vessels with xdyn
on 12345/12346). The default model loaded by `entity_manager` is `model.sdf`
(overridable via `sdf_file`).
