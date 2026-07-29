# Radar Sensor

The `radar_sensor` is a custom Gazebo sensor that simulates a marine surface radar. It uses point cloud data from an onboard 2D LiDAR and applies a Point Spread Function (PSF) to produce a realistic radar-style image, mimicking the blurring and smearing artefacts seen on real radar displays.

---

## How It Works

```
LiDAR (gpu_lidar)
      │
      │  Gazebo transport point cloud
      ▼
RadarSensor::OnPointCloud()
      │
      │  decode x, y from PointCloudPacked
      ▼
RadarSensor::SimulatePSF()
      │  normalise points to grid
      │  apply elliptical PSF per point
      │  (ellipse size scales with range)
      ▼
Published topics
  ├── /vessel/radar_sensor/radar/image       (PSF-blurred radar image)
  ├── /vessel/radar_sensor/radar/lidar_image (raw LiDAR points as image)
  └── /vessel/radar_sensor/radar/detections  (structured RadarScan detections)
```

The PSF ellipse orientation follows the azimuth direction from the radar origin, and its size scales with the range of each detection, closer targets appear sharper, distant targets smear more, as on a real radar.

---

## SDF Setup

The radar sensor requires a lidar sensor on the same link to provide point cloud input. Both must be declared inside the same `<link>`:

```xml
<link name="base_link">

    <!-- 2D LiDAR — required input for the radar -->
    <sensor name="lidar_sensor" type="gpu_lidar">
        <pose>0 0 0.5 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>10.0</update_rate>
        <visualize>true</visualize>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>1.0</min>
                <max>100.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
    </sensor>

    <!-- Radar sensor — uses LiDAR point cloud -->
    <sensor name="radar_sensor" type="custom" gz:type="radar">
        <always_on>true</always_on>
        <update_rate>1</update_rate>
        <is_on>true</is_on>

        <!-- Gazebo transport topic of the LiDAR above -->
        <!-- pattern: world/<world_name>/model/<vessel_name>/link/base_link/sensor/lidar_sensor/scan/points -->
        <lidar_gz_topic>
            world/lotusim/model/<vessel_name>/link/base_link/sensor/lidar_sensor/scan/points
        </lidar_gz_topic>

        <!-- Detection range -->
        <min_range>1.0</min_range>
        <max_range>100.0</max_range>

        <!-- PSF parameters -->
        <psf_ellipse_width>15000</psf_ellipse_width>
        <psf_ellipse_height>5000</psf_ellipse_height>
        <psf_range_factor>0.2</psf_range_factor>

        <!-- Sensor noise -->
        <noise_sigma>0.01</noise_sigma>
        <noise_amplitude>0.01</noise_amplitude>
    </sensor>

</link>
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `update_rate` | Hz | — | How often the radar image is generated |
| `is_on` | bool | true | Initial sensor state |
| `min_range` | m | 1.0 | Minimum detection range |
| `max_range` | m | 100.0 | Maximum detection range |
| `psf_ellipse_width` | px | 15000 | PSF ellipse width — controls azimuth smear |
| `psf_ellipse_height` | px | 5000 | PSF ellipse height — controls range smear |
| `psf_range_factor` | float | 0.2 | Scales PSF ellipse size with target range |
| `noise_sigma` | float | 0.0 | Gaussian noise standard deviation |
| `noise_amplitude` | float | 0.0 | Gaussian noise amplitude |

**Tuning the PSF:**
- Increase `psf_ellipse_width` for more azimuth smearing (wider sweeps per target)
- Increase `psf_ellipse_height` for more range smearing
- Increase `psf_range_factor` to make far targets smear more than near ones

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/<vessel>/radar_sensor/radar/image` | `sensor_msgs/Image` | PSF-blurred radar display (bgr8) |
| `/<vessel>/radar_sensor/radar/lidar_image` | `sensor_msgs/Image` | Raw LiDAR points rendered as image (bgr8) |
| `/<vessel>/radar_sensor/radar/detections` | `radar_msgs/RadarScan` | Structured detections with range, azimuth, amplitude |

For a vessel named `commando_center`, the topics are:
```
/commando_center/radar_sensor/radar/image
/commando_center/radar_sensor/radar/lidar_image
/commando_center/radar_sensor/radar/detections
```

---

## Viewing Results in RViz2

**1. Launch the simulation**, then open RViz2:

```bash
ros2 run rviz2 rviz2
```

**2. Add an Image display for the radar image:**

- In the RViz2 panel, click **Add** → **By topic** → select `/<vessel>/radar_sensor/radar/image` → **Image**
- Or click **Add** → **By display type** → **Image**, then set the topic manually

**3. Verify the LiDAR is running:**

The radar will produce no output if the LiDAR point cloud is not arriving. Check:

```bash
ros2 topic echo /<vessel>/radar_sensor/radar/detections --no-arr
```

---

## Activating and Deactivating

The radar sensor exposes a ROS 2 service to toggle it on and off at runtime:

```bash
ros2 service call /<vessel>/radar_sensor/change_state \
    lotusim_sensor_msgs/srv/ActivateSensor "{activate: false}"

ros2 service call /<vessel>/radar_sensor/change_state \
    lotusim_sensor_msgs/srv/ActivateSensor "{activate: true}"
```

When deactivated, the sensor stops processing point clouds and stops publishing on all topics.

If the power subsystem is enabled on the vessel, the radar consumer can also be deactivated automatically by the power manager during load shedding.