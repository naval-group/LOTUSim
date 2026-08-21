#!/usr/bin/env python3
"""
seafloor_viewer.py
 
Loads a LOTUSim seafloor heightmap PNG and shows vessels moving on top of it. A side legend lists each tracked vessel's name
and its live DVL velocity output (body-frame speed).
 
Coordinate mapping matches DopplerVelocityLog::SeafloorElevation():
    u = (world_x - origin_x) / size_x
    v = (world_y - origin_y) / size_y
    image row 0 (top)    <-> world_y = origin_y + size_y
    image row H-1 (bottom) <-> world_y = origin_y
 
Vessel positions are read live from the /lotusim/poses topic (lotusim_msgs/msg/VesselPositionArray).
 
Vessel speed is read live from each vessel's DVL topic (lotusim_sensor_msgs/msg/DVL), using the body-frame velocity components
(velocity.x, velocity.y, velocity.z).
 
If the seafloor origin isn't set explicitly, the viewer auto-centers the same way:
    origin = first_reported_position_of_anchor_vessel - size / 2.
 
Usage:
    python3 seafloor_viewer.py \
        --seafloor /path/to/seafloor.png \
        --size-x 1000 --size-y 1000 \
        --vessels lrauv_0 lrauv_1
        # add --origin-x/--origin-y if your SDF sets them explicitly
 
"""
 
import argparse
import random
 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PIL import Image
 
import rclpy
from rclpy.node import Node
 
from lotusim_msgs.msg import VesselPositionArray
from lotusim_sensor_msgs.msg import DVL
 
POSITIONS_FIELD = "vessels"
 
BG_COLOR = "#001233"
FG_COLOR = "white"
 
 
class VesselPoseSource(Node):
    """Subscribes to /lotusim/poses and keeps the latest position per vessel."""
 
    def __init__(self, vessel_filter):
        super().__init__("seafloor_viewer")
        self.vessel_filter = set(vessel_filter) if vessel_filter else None
        self._latest = {}  # vessel_name -> (x, y)
        self.create_subscription(VesselPositionArray, "/lotusim/poses", self._callback, 10)
 
    def _callback(self, msg):
        entries = getattr(msg, POSITIONS_FIELD)
        for entry in entries:
            name = entry.vessel_name
            if self.vessel_filter is not None and name not in self.vessel_filter:
                continue
            p = entry.pose.position
            self._latest[name] = (p.x, p.y)
 
    def get_positions(self):
        """Returns dict: vessel_name -> (x, y). Only vessels seen so far are included."""
        return dict(self._latest)
 
 
class DVLVelocitySource(Node):
    """Discovers each vessel's DVL topic automatically and tracks its reported body-frame velocity"""
 
    def __init__(self, vessel_names):
        super().__init__("dvl_velocity_source")
        self._vessel_names = list(vessel_names)
        self._subscribed = set()
        self._latest = {}  # vessel_name -> (vx, vy, vz, bottom_lock)
        self._debug_frame_count = 0
        self._debug_printed = False
 
    def try_discover(self):
        all_topics = self.get_topic_names_and_types()
        for name in self._vessel_names:
            if name in self._subscribed:
                continue
            for topic_name, types in all_topics:
                segments = topic_name.strip("/").split("/")
                if name in segments and segments[-1] == "dvl":
                    self.create_subscription(
                        DVL, topic_name,
                        lambda msg, n=name: self._callback(n, msg),
                        10
                    )
                    self._subscribed.add(name)
                    print(f"[seafloor_viewer] discovered DVL topic for "
                          f"'{name}': {topic_name} (types={types})")
                    for _ in range(20):
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if name in self._latest:
                            break
                    break
 
        # one-time diagnostic if nothing found after ~3 sec
        self._debug_frame_count += 1
        missing = set(self._vessel_names) - self._subscribed
        if missing and self._debug_frame_count > 15 and not self._debug_printed:
            self._debug_printed = True
            candidates = [(n, t) for n, t in all_topics if "dvl" in n.lower()]
            print(f"[seafloor_viewer] DEBUG: still missing DVL for {missing}. "
                  f"All topics containing 'dvl' currently visible to this node:")
            if not candidates:
                print(" none -- this node's graph cache sees no 'dvl' topics at all.")
            for n, t in candidates:
                print(f"  {n}  types={t}")
 
    def _callback(self, vessel_name, msg):
        if vessel_name not in self._latest:
            print(f"[seafloor_viewer] first DVL message received for "
                  f"'{vessel_name}': velocity=({msg.velocity.x:.3f}, "
                  f"{msg.velocity.y:.3f}, {msg.velocity.z:.3f}), "
                  f"bottom_lock={msg.bottom_lock}")
        v = msg.velocity
        self._latest[vessel_name] = (v.x, v.y, v.z, msg.bottom_lock)
 
    def get_velocities(self):
        """Returns dict: vessel_name -> (vx, vy, vz, bottom_lock) in m/s."""
        return dict(self._latest)
 
 
def world_to_extent(origin_x, origin_y, size_x, size_y):
    return [origin_x, origin_x + size_x, origin_y, origin_y + size_y]
 
 
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seafloor", required=True, help="Path to seafloor PNG")
    parser.add_argument("--size-x", type=float, default=1000.0)
    parser.add_argument("--size-y", type=float, default=1000.0)
    parser.add_argument("--origin-x", type=float, default=None,
                         help="Explicit origin. Omit to auto-center like the "
                              "C++ sensor does (origin = anchor vessel's "
                              "first position - size/2).")
    parser.add_argument("--origin-y", type=float, default=None)
    parser.add_argument("--vessels", nargs="+", required=True,
                         help="Vessel names to track. Each vessel's DVL topic is discovered automatically.")
    parser.add_argument("--anchor-vessel", default=None,
                         help="Vessel used to auto-center the seafloor when --origin-x/y aren't given."
                              "Defaults to the first name in --vessels.")
    parser.add_argument("--rate-hz", type=float, default=5.0, help="Plot refresh rate")
    args = parser.parse_args()
 
    origin_explicit = args.origin_x is not None and args.origin_y is not None
    anchor_name = args.anchor_vessel or args.vessels[0]
 
    vessel_colors = {
        name: (random.random(), random.random(), random.random())
        for name in args.vessels
    }
 
    # ── Load seafloor image ──────────────────────────────────────────────
    img = Image.open(args.seafloor).convert("L")  # grayscale, matches gz::common::Image usage
    img_arr = np.asarray(img)
 
    # ── ROS2 setup ────────────────────────────────────────────────────────
    rclpy.init()
    pose_source = VesselPoseSource(args.vessels)
    velocity_source = DVLVelocitySource(args.vessels)
 
    origin = {"x": args.origin_x, "y": args.origin_y, "centered": origin_explicit}
 
    def try_auto_center():
        """Mirrors LoadSeafloor/UpdateSensor's auto-center-on-first-position logic."""
        if origin["centered"]:
            return
        positions = pose_source.get_positions()
        if anchor_name not in positions:
            return
        px, py = positions[anchor_name]
        origin["x"] = px - args.size_x / 2.0
        origin["y"] = py - args.size_y / 2.0
        origin["centered"] = True
        print(f"[seafloor_viewer] auto-centered on '{anchor_name}', "
              f"origin=({origin['x']:.2f}, {origin['y']:.2f})")
 
    if not origin_explicit:
        import time as _time
        deadline = _time.time() + 10.0
        while not origin["centered"] and _time.time() < deadline:
            rclpy.spin_once(pose_source, timeout_sec=0.2)
            try_auto_center()
        if not origin["centered"]:
            print("[seafloor_viewer] WARNING: no pose received within 10s to "
                  "auto-center on -- defaulting origin to (0, 0). Pass "
                  "--origin-x/--origin-y explicitly to avoid this.")
            origin["x"], origin["y"] = 0.0, 0.0
 
    extent = world_to_extent(origin["x"], origin["y"], args.size_x, args.size_y)
 
    # map on the left, legend panel on the right
    fig, (ax_map, ax_legend) = plt.subplots(
        1, 2, figsize=(11, 8), gridspec_kw={"width_ratios": [3, 1]})
    fig.patch.set_facecolor(BG_COLOR)
    ax_map.set_facecolor(BG_COLOR)
    ax_legend.set_facecolor(BG_COLOR)
 
    ax_map.imshow(img_arr, cmap="gray", extent=extent, origin="upper")
    ax_map.set_xlabel("world X (m)", color=FG_COLOR)
    ax_map.set_ylabel("world Y (m)", color=FG_COLOR)
    ax_map.set_title("Seafloor + live vessel tracking", color=FG_COLOR)
    ax_map.tick_params(axis="both", colors=FG_COLOR)
    for spine in ax_map.spines.values():
        spine.set_color(FG_COLOR)
 
    scatter = ax_map.scatter([], [], s=60, zorder=5, edgecolors=FG_COLOR)
 
    ax_legend.axis("off")
    ax_legend.set_title("Vessels (DVL speed)", loc="left", color=FG_COLOR)
 
    def update(_frame):
        rclpy.spin_once(pose_source, timeout_sec=0.02)
        rclpy.spin_once(velocity_source, timeout_sec=0.02)
        velocity_source.try_discover()
 
        positions = pose_source.get_positions()
        velocities = velocity_source.get_velocities()
 
        names, xy, colors = [], [], []
        for name in args.vessels:
            if name in positions:
                names.append(name)
                xy.append(positions[name])
                colors.append(vessel_colors[name])
 
        xy_arr = np.array(xy) if xy else np.empty((0, 2))
        scatter.set_offsets(xy_arr)
        scatter.set_color(colors if colors else [])
 
        ax_legend.cla()
        ax_legend.set_facecolor(BG_COLOR)
        ax_legend.axis("off")
        ax_legend.set_title("Vessels (DVL speed)", loc="left", color=FG_COLOR)
        line_height = 0.9 / max(len(args.vessels), 1)
        for i, name in enumerate(args.vessels):
            y = 0.95 - i * line_height
            if name in velocities:
                vx, vy, vz, bottom_lock = velocities[name]
                speed = float(np.sqrt(vx**2 + vy**2 + vz**2))
                lock_str = "bottom lock" if bottom_lock else "no lock"
                info_str = f"{speed:.2f} m/s\n({lock_str})"
            else:
                info_str = "no data yet"
            ax_legend.text(0.02, y, "\u25cf", color=vessel_colors[name],
                            fontsize=14, va="center", transform=ax_legend.transAxes)
            ax_legend.text(0.18, y, f"{name}\n{info_str}", color=FG_COLOR,
                            fontsize=9, va="center", transform=ax_legend.transAxes)
 
        return scatter,
 
    interval_ms = int(1000.0 / args.rate_hz)
    anim = FuncAnimation(fig, update, interval=interval_ms, blit=False)
 
    try:
        plt.show()
    finally:
        pose_source.destroy_node()
        velocity_source.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 