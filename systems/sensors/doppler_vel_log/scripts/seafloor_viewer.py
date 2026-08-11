#!/usr/bin/env python3
"""
Loads a LOTUSim seafloor heightmap PNG and shows one or more vessels moving on top of it, as red dots. Hovering over a dot shows the vessel's name.

Coordinate mapping matches DopplerVelocityLog::SeafloorElevation() exactly:
    u = (world_x - origin_x) / size_x
    v = (world_y - origin_y) / size_y
    image row 0 (top)    <-> world_y = origin_y + size_y
    image row H-1 (bottom) <-> world_y = origin_y

Vessel positions are read live from the /lotusim/poses topic (lotusim_msgs/msg/VesselPositionArray). If the seafloor origin isn't set explicitly, it auto-centers:
origin = first_reported_position_of_anchor_vessel - size / 2.

Usage:
    python3 seafloor_viewer.py \\
        --seafloor /path/to/seafloor.png \\
        --size-x 1000 --size-y 1000 \\
        --vessels lrauv_1 lrauv_2 \\
        # add --origin-x/--origin-y if your SDF sets them explicitly

Requires: rclpy, matplotlib, numpy, Pillow
"""

import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PIL import Image

import rclpy
from rclpy.node import Node

from lotusim_msgs.msg import VesselPositionArray

POSITIONS_FIELD = "vessels"


class VesselPoseSource(Node):
    """Subscribes to /lotusim/poses and keeps the latest position per vessel."""

    def __init__(self, vessel_filter):
        super().__init__("seafloor_viewer")
        self.vessel_filter = set(vessel_filter) if vessel_filter else None
        self._latest = {}  # vessel_name -> (x, y)
        self.create_subscription(
            VesselPositionArray, "/lotusim/poses", self._callback, 10)

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


def world_to_extent(origin_x, origin_y, size_x, size_y):
    """extent=[xmin, xmax, ymin, ymax] for imshow, matching origin='upper'."""
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
    parser.add_argument("--vessels", nargs="+", default=None,
                         help="Vessel names to track. Omit to track every "
                              "vessel seen on /lotusim/poses.")
    parser.add_argument("--anchor-vessel", default=None,
                         help="Vessel used to auto-center the seafloor when "
                              "--origin-x/y aren't given. Defaults to the "
                              "first name in --vessels, or the first vessel "
                              "seen on the topic if --vessels is omitted.")
    parser.add_argument("--rate-hz", type=float, default=5.0,
                         help="Plot refresh rate")
    args = parser.parse_args()

    origin_explicit = args.origin_x is not None and args.origin_y is not None
    anchor_name = args.anchor_vessel or (args.vessels[0] if args.vessels else None)

    # ── Load seafloor image ──────────────────────────────────────────────
    img = Image.open(args.seafloor).convert("L")  # grayscale, matches gz::common::Image usage
    img_arr = np.asarray(img)

    # ── ROS2 setup ────────────────────────────────────────────────────────
    rclpy.init()
    pose_source = VesselPoseSource(args.vessels)

    origin = {"x": args.origin_x, "y": args.origin_y, "centered": origin_explicit}

    def try_auto_center():
        """Mirrors LoadSeafloor/UpdateSensor's auto-center-on-first-position logic."""
        if origin["centered"]:
            return
        positions = pose_source.get_positions()
        name = anchor_name or (next(iter(positions), None))
        if name is None or name not in positions:
            return
        px, py = positions[name]
        origin["x"] = px - args.size_x / 2.0
        origin["y"] = py - args.size_y / 2.0
        origin["centered"] = True
        print(f"[seafloor_viewer] auto-centered on '{name}', "
              f"origin=({origin['x']:.2f}, {origin['y']:.2f})")

    if not origin_explicit:
        # Spin briefly waiting for the anchor vessel's first pose so the map
        # doesn't render at the wrong place before centering.
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

    # ── Matplotlib figure ────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img_arr, cmap="gray", extent=extent, origin="upper")
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    ax.set_title("Seafloor + live vessel tracking")

    scatter = ax.scatter([], [], c="red", s=60, zorder=5, edgecolors="black")
    annot = ax.annotate(
        "", xy=(0, 0), xytext=(12, 12), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="white", ec="black", alpha=0.9),
        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    # keep last-known plotted points + names in sync for hover lookup
    state = {"names": [], "xy": np.empty((0, 2))}

    def update(_frame):
        rclpy.spin_once(pose_source, timeout_sec=0.0)
        positions = pose_source.get_positions()

        names, xy = [], []
        for name, pos in positions.items():
            if pos is not None:
                names.append(name)
                xy.append(pos)

        xy = np.array(xy) if xy else np.empty((0, 2))
        scatter.set_offsets(xy)
        state["names"] = names
        state["xy"] = xy
        return scatter,

    def on_move(event):
        if event.inaxes != ax or state["xy"].size == 0:
            annot.set_visible(False)
            fig.canvas.draw_idle()
            return

        # distance in display (pixel) space so hover radius feels consistent
        disp_xy = ax.transData.transform(state["xy"])
        mouse_disp = np.array([event.x, event.y])
        dists = np.hypot(*(disp_xy - mouse_disp).T)
        idx = np.argmin(dists)

        HOVER_RADIUS_PX = 15
        if dists[idx] <= HOVER_RADIUS_PX:
            annot.xy = state["xy"][idx]
            annot.set_text(state["names"][idx])
            annot.set_visible(True)
        else:
            annot.set_visible(False)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_move)

    interval_ms = int(1000.0 / args.rate_hz)
    anim = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    try:
        plt.show()
    finally:
        pose_source.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()