#!/usr/bin/env python3
"""
sonar_viewer.py

Sector scan style active sonar
Shows:
  - the active 45-deg sector, shaded
  - a sweep line at the sector's center bearing
  - contact blips at (bearing, range), with name shown on hover
  - a short fade trail of recent contacts

Bearing convention matches the sensor: nautical, clockwise from bow (0 deg = straight ahead, 90 deg = to starboard/right)

Usage:
    python3 sonar_viewer.py --topic /lotusim/lrauv_0/active_sonar/scan

"""

import argparse
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
import threading

from lotusim_sensor_msgs.msg import SonarScan

# How many past scans to keep on screen
TRAIL_LENGTH = 5
HOVER_RADIUS_PX = 15

BG_COLOR =  "#001233"
FG_COLOR = "white"
NEAR_COLOR = np.array([1.0, 0.0, 0.0])  # red, range < RANGE_THRESHOLD_M
FAR_COLOR = np.array([0.0, 1.0, 0.0])   # green, range >= RANGE_THRESHOLD_M
RANGE_THRESHOLD_M = 80.0


class SonarScanSource(Node):
    def __init__(self, topic):
        super().__init__("sonar_viewer")
        self.history = deque(maxlen=TRAIL_LENGTH)
        self.latest_sector_info = None  # (current_sector, center_deg, width_deg, max_range)
        self.vessel_name = None
        self.sensor_name = None
        self.lock = threading.Lock()
        self.create_subscription(SonarScan, topic, self._callback, 10)

    def _callback(self, msg):
        contacts = [(c.bearing_deg, c.range, c.target_name) for c in msg.contacts]
        frame_parts = msg.header.frame_id.split("/", 1)
        with self.lock:
            self.history.appendleft(contacts)
            self.latest_sector_info = (
                msg.current_sector, msg.sector_center_deg,
                msg.sector_width_deg, msg.max_range)
            self.vessel_name = frame_parts[0] if frame_parts else msg.header.frame_id
            self.sensor_name = frame_parts[1] if len(frame_parts) > 1 else ""


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default=None,
                         help="Full topic path, e.g. /lrauv_0/sonar_0/scan")
    parser.add_argument("--vessel", default=None,
                         help="Vessel name, used to build the default topic "
                              "if --topic isn't given")
    parser.add_argument("--sensor", default=None,
                         help="Sensor name, used to build the default topic "
                              "if --topic isn't given")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    args = parser.parse_args()

    if args.topic:
        topic = args.topic
    elif args.vessel and args.sensor:
        topic = f"{args.vessel}/{args.sensor}/scan"
    else:
        parser.error("Provide either --topic, or both --vessel and --sensor")

    rclpy.init()
    source = SonarScanSource(topic)

    spin_thread = threading.Thread(target=rclpy.spin, args=(source,), daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(7, 7))
    fig.patch.set_facecolor(BG_COLOR)
    ax = fig.add_subplot(projection="polar")
    ax.set_facecolor(BG_COLOR)
    ax.set_theta_zero_location("N")  # 0 deg = straight up (bow)
    ax.set_theta_direction(-1)       # clockwise, matching nautical bearing

    ax.tick_params(axis="both", colors=FG_COLOR)
    ax.grid(color=FG_COLOR, alpha=0.3)
    for spine in ax.spines.values():
        spine.set_color(FG_COLOR)

    max_range_guess = 500.0  # placeholder until first scan arrives
    ax.set_ylim(0, max_range_guess)
    fig.suptitle("Active sonar - sector scan", color=FG_COLOR, fontsize=13, y=0.98)
    fig.text(0.5, 0.94, topic, color=FG_COLOR, ha="center", fontsize=9, alpha=0.75)

    sector_wedge, = ax.fill([0, 0], [0, 0], color=FG_COLOR, alpha=0.15, zorder=1)
    sweep_line, = ax.plot([0, 0], [0, max_range_guess], color=FG_COLOR, linewidth=2, zorder=3)
    scatter = ax.scatter([], [], c="red", s=50, zorder=5, edgecolors=FG_COLOR)
    annot = ax.annotate(
        "", xy=(0, 0), xytext=(12, 12), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="white", ec="black", alpha=0.9),
        color="black",
        arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    state = {"theta": np.empty(0), "r": np.empty(0), "names": []}

    def update(_frame):
        with source.lock:
            if source.latest_sector_info is None:
                return sector_wedge, sweep_line, scatter
            sector_info = source.latest_sector_info
            history_snapshot = list(source.history)
            vessel_name = source.vessel_name

        _, center_deg, width_deg, max_range = sector_info
        ax.set_ylim(0, max_range)

        # Sector
        lo = np.radians(center_deg - width_deg / 2.0)
        hi = np.radians(center_deg + width_deg / 2.0)
        wedge_theta = np.linspace(lo, hi, 20)
        wedge_theta = np.concatenate(([0], wedge_theta, [0]))  # back to center
        wedge_r = np.concatenate(([0], np.full(20, max_range), [0]))
        sector_wedge.set_xy(np.column_stack([wedge_theta, wedge_r]))

        # sweep line at sector center
        center_rad = np.radians(center_deg)
        sweep_line.set_data([center_rad, center_rad], [0, max_range])

        # contacts with fade trail
        all_theta, all_r, all_colors, names = [], [], [], []
        n = len(history_snapshot)
        for age, contacts in enumerate(history_snapshot):
            alpha = 1.0 - age / max(n, 1)
            for bearing_deg, rng, name in contacts:
                base_color = NEAR_COLOR if rng < RANGE_THRESHOLD_M else FAR_COLOR
                all_theta.append(np.radians(bearing_deg))
                all_r.append(rng)
                all_colors.append((*base_color, alpha))
                names.append(name)

        if all_theta:
            offsets = np.column_stack([all_theta, all_r])
            scatter.set_offsets(offsets)
            scatter.set_color(all_colors)
        else:
            scatter.set_offsets(np.empty((0, 2)))

        state["theta"] = np.array(all_theta)
        state["r"] = np.array(all_r)
        state["names"] = names
        state["vessel"] = vessel_name 

        return sector_wedge, sweep_line, scatter

    def on_move(event):
        if event.inaxes != ax or state["theta"].size == 0:
            annot.set_visible(False)
            fig.canvas.draw_idle()
            return

        points = np.column_stack([state["theta"], state["r"]])
        disp_xy = ax.transData.transform(points)
        mouse_disp = np.array([event.x, event.y])
        dists = np.hypot(*(disp_xy - mouse_disp).T)
        idx = np.argmin(dists)

        if dists[idx] <= HOVER_RADIUS_PX:
            annot.xy = (state["theta"][idx], state["r"][idx])
            vessel_label = state["vessel"] or "?" 
            annot.set_text(
                f"{state['names'][idx]}\n"
                f"{np.degrees(state['theta'][idx]):.0f}\u00b0, "
                f"{state['r'][idx]:.1f} m\n"
                f"sensor: {vessel_label}")
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
        source.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()