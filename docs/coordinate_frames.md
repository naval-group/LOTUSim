# Coordinate frames and orientation conventions

Reference for which frame is used where, which conversion is applied where,
and how to set a Unity prefab's mesh orientation.

Invariant: every vehicle publishes its pose on `/<world>/poses` in world ENU
with a body x-forward frame (REP-103), regardless of physics backend.

## 1. Frames

| Frame | Axes | Handedness | Body forward |
|---|---|---|---|
| xdyn | world NED: X=North, Y=East, Z=Down | right | body +X (FRD: x fwd, y starboard, z down) |
| Gazebo / ECM / `/poses` | world ENU: X=East, Y=North, Z=Up | right | body +X (FLU, REP-103) |
| Unity | X=East, Y=Up, Z=North | left | object +Z (`transform.forward`) |
| Scenario waypoints | geographic lat/lon | - | - |

- Waypoints are projected to local ENU metres in
  `lotusim_sdk/tasks/waypoint_follower.py::_project_waypoints`, using the
  world's geographic origin. Same origin Gazebo uses, so waypoints and
  `/poses` share a frame.

## 2. Conversions along the pipeline

### 2.1 xdyn <-> Gazebo (`systems/physics_engine_interface/src/vessel_information.cpp`)

`XdynWebsocket` converts every state it exchanges with xdyn through
`VesselInformation::to_xdyn()` and `VesselInformation::from_xdyn()`, which
use the frame helpers of `lotusim_common` (`common.hpp`).

An attitude maps body vectors to world vectors. A change of convention needs
the world basis change on the left and the body basis change on the right:

```
R_enu_flu = M_world(NED->ENU) . R_ned_frd . M_body(FRD->FLU)^-1
```

- `common::kWorldEnuNed` - world basis change, swaps X and Y and flips Z
- `common::kBodyFluFrd` - body basis change, flips Y and Z (180 deg about body X)
- `common::poseChangeFrame(pose, kWorldEnuNed, kBodyFluFrd)` applies both to a
  pose.
- Velocities: xdyn's u,v,w and p,q,r are BODY frame, the ECM's are WORLD
  frame. `worldVelToTargetBody` and `targetBodyVelToWorld` rotate them, with
  the angular velocity handled as a pseudo-vector.
- Conjugating by the world change alone (same rotation both sides) would apply
  the world map to the body frame too, putting xdyn's body x (forward) onto
  Gazebo body y. Both changes are needed.
- A plugin-side current (Gauss-Markov, or a replayed measured profile) is
  subtracted from the world velocity before the conversion, so xdyn steps on
  the velocity relative to the water, and added back to the state it returns.

### 2.2 Kinematic vehicles (`systems/physics_engine_interface/src/kinematic_interface.cpp`)

```cpp
pose.SetX(pose.X() + (u * std::cos(yaw) + cx) * dt_s);
pose.SetY(pose.Y() + (u * std::sin(yaw) + cy) * dt_s);
```

Unicycle model, forward along body +X. Already REP-103, not affected by this
change.

### 2.3 Gazebo -> ROS (`systems/render_interface/src/ros_interface.cpp:77-83`)

ECM pose is copied into `VesselPosition` unchanged. No conversion.

### 2.4 ENU -> NED for control (`lotusim_sdk/control/frames.py`)

`enu_quat_to_ned_euler` inverts the attitude change of 2.1 and must be kept in
step with it:

```python
q_ned = q_ned_to_enu^-1 * q_enu * q_frd_to_flu
```

- Consumers: `tasks/control.py`, `tasks/guidance.py`, `bluerov_gnc`,
  `lrauv_gnc`. This is the only ENU<->NED quaternion math in Python.
- Because it is an exact inverse of 2.1, the NED state the GNC sees does not
  change when 2.1 changes. No guidance/control retuning needed.

### 2.5 ROS -> Unity (`Assets/Scripts/lotusim_interface/common.cs:34-52`)

```csharp
position = (x, z, y);          // ENU -> Unity Y-up
rotation = (-x, -z, -y, w);    // right-handed -> left-handed
```

- For a level vehicle this reduces to `unityEuler.y = -rosYaw`.
- Applied exactly once per vessel, by one of two consumers:
  - `LotusimConnector.UpdateVesselPoses` (line 330), for vessels without a
    `RendererPosesWaypointFollower`.
  - `RendererPosesWaypointFollower.OnVesselPositionsReceived` (line 69), for
    vessels with one. `LotusimConnector` skips those (line 328) so the two
    do not both drive the transform.
- Known defect, ROS2 path unaffected: `TcpIpInterface.UpdateVesselPoses`
  (line 325) stores an already-converted pose into `m_vesselPoses`, which
  `LotusimConnector` then converts a second time. `RosInterface` stores the
  raw pose correctly. Only reachable if `selectedInterfaceType` is `TCP`;
  default is `ROS2`.

## 3. Configuring a Unity prefab

The root object receives the converted world attitude (`unityEuler.y = -psi`).
A child pivot node (`model_pivot`, or `bluerov2_heavy` on the BlueROV) holds a
fixed rotation aligning the mesh's own forward axis with the vehicle's
physical forward.

Since section 1's invariant makes psi mean the same thing for every backend,
this value depends only on how the mesh was authored: one value per vehicle
type, set on the base prefab. Variants (`*_inspection`) inherit it and must
not override it.

| Prefab | Pivot node | Rotation |
|---|---|---|
| `wamv.prefab` | `model_pivot` | Euler (0, 180, 0) |
| `bluerov.prefab` | `bluerov2_heavy` | Euler (0, 90, 90) |
| `x500.prefab` | `model_pivot` | Euler (0, 90, 0) |
| `lrauv.prefab` | `lrauv` | Euler (0, 180, 0) |
| `mine.prefab` | needs pivot, see below | Euler (0, 90, 0) once a pivot exists |
| `pha` (`AdditionnalShips/Prefab/S10.prefab`) | needs pivot, see below | Euler (0, 90, 0) once a pivot exists |
| `commando` (`models/generic/commando1.prefab`) | needs pivot, see below | Euler (0, 90, 0) once a pivot exists |
| `fremm` (`models/boat/fremm.dae`) | needs prefab + pivot, see below | Euler (0, 90, 0) once a pivot exists |

- Every value above is the pre-existing prefab rotation plus 90 deg about Y.
  The old xdyn convention needed no yaw compensation; the corrected one needs
  90 deg, so the offset is uniform across the fleet.

### Prefabs with no pivot node

`mine`, `pha`, `commando` and `fremm` put their mesh at the prefab root
(`fremm` is not a prefab; the Addressable points at the `.dae` directly). The
root transform is overwritten every frame by the incoming pose
(`Instantiate(prefab, pos, rot)`, then `SetPositionAndRotation`), so a
rotation written there is discarded. Steps:

1. Open the prefab, right-click the root, Create Empty, name it
   `model_pivot`.
2. Drag the root's existing children into it.
3. Set `model_pivot` Rotation Y = 90, position 0,0,0. Save.

For `fremm`: first wrap the `.dae` in a new `fremm.prefab` with that pivot,
then repoint the `fremm` Addressable entry at the prefab (address string
unchanged).

These four are spawned by `all_vehicles_current_demo.json` and the four
`multi_vehicle_examples/*_drift_only.json` scenarios.

### Adding a new vehicle

1. Spawn it in any scenario, let it move along a known heading.
2. Select its pivot node in the Hierarchy while playing.
3. Adjust Transform Rotation until the nose matches the direction of travel.
4. Write that value into the base prefab. Leave variants inheriting.

Rotation about the nose axis itself is a roll: it changes which way "up"
faces and has no effect on heading.

### If a vehicle renders 90 or 180 degrees off

Check in this order:

1. The pivot value, live in the Editor (steps 2-3 above) - confirms the node
   is actually the mesh's parent and the file value is what loaded.
2. That the variant does not override the base value.
3. That 2.1 and 2.4 still match each other. They are inverses; changing one
   alone silently rotates every xdyn vessel by 90 deg while leaving kinematic
   ones correct.
