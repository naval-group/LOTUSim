#!/usr/bin/env python3
# Copyright (c) 2026 Naval Group
# SPDX-License-Identifier: EPL-2.0
#
# Usage:
#   python3 power_monitor.py
#   python3 power_monitor.py --max-history 120
#
# Subscribes to:
#   /lotusim/<vessel_name>/power_status  (lotusim_msgs/msg/PowerStatus)

import argparse
import threading
import time
from collections import deque

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from lotusim_msgs.msg import PowerStatus


# ── constants ────────────────────────────────────────────────────────────────

MAX_VESSELS = 5
COLORS = ["#2196F3", "#4CAF50", "#FF5722", "#9C27B0", "#FF9800"]


# ── data store ───────────────────────────────────────────────────────────────


class VesselData:
    def __init__(self, max_history: int):
        self.times: deque[float] = deque(maxlen=max_history)
        self.socs: deque[float] = deque(maxlen=max_history)
        self.active_name: str = ""
        self.active_type: str = ""
        self.voltage: float = 0.0

    def push(self, t: float, msg: PowerStatus) -> None:
        self.times.append(t)
        self.socs.append(float(msg.active_provider_soc))
        self.active_name = msg.active_provider_name
        self.active_type = msg.active_provider_type
        self.voltage = float(msg.active_provider_voltage)


# ── ROS2 node ────────────────────────────────────────────────────────────────


class PowerMonitorNode(Node):
    def __init__(self, max_history: int):
        super().__init__("power_monitor")
        self._lock = threading.Lock()
        self._data: dict[str, VesselData] = {}
        self._colors: dict[str, str] = {}
        self._t0: float | None = None
        self._max_history = max_history
        self._known_topics: set[str] = set()
        self._subs: dict[str, object] = {}
        self.get_logger().info("PowerMonitorNode started — waiting for vessels...")

    def refresh_subscriptions(self) -> None:
        for topic, types in self.get_topic_names_and_types():
            if not topic.endswith("/power_status"):
                continue
            if topic in self._known_topics:
                continue
            if "PowerStatus" not in str(types):
                continue
            if len(self._subs) >= MAX_VESSELS:
                self.get_logger().warn(
                    f"MAX_VESSELS={MAX_VESSELS} reached, ignoring {topic}"
                )
                continue
            self._known_topics.add(topic)
            self._subs[topic] = self.create_subscription(
                PowerStatus, topic, lambda msg, t=topic: self._cb(t, msg), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def _cb(self, _topic: str, msg: PowerStatus) -> None:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        vessel = msg.vessel_name
        with self._lock:
            if vessel not in self._data:
                idx = len(self._data)
                self._data[vessel] = VesselData(self._max_history)
                self._colors[vessel] = COLORS[idx % len(COLORS)]
                self.get_logger().info(f"New vessel tracked: {vessel}")
            self._data[vessel].push(t, msg)

    def snapshot(self) -> dict[str, tuple[VesselData, str]]:
        with self._lock:
            return {v: (d, self._colors[v]) for v, d in self._data.items()}


# ── plot ─────────────────────────────────────────────────────────────────────


class PowerMonitorPlot:
    def __init__(self, node: PowerMonitorNode):
        self._node = node
        self._lines: dict[str, object] = {}

        self._fig, self._ax = plt.subplots(figsize=(13, 5))
        self._fig.patch.set_facecolor("#1e1e2e")
        self._ax.set_facecolor("#1e1e2e")

        self._ax.set_title(
            "LOTUSim — Power Monitor", color="white", fontsize=14, pad=12
        )
        self._ax.set_xlabel("Time (s)", color="#aaaaaa")
        self._ax.set_ylabel("State of Charge / Fuel ratio (0–1)", color="#aaaaaa")
        self._ax.tick_params(colors="#aaaaaa")
        for spine in self._ax.spines.values():
            spine.set_edgecolor("#444444")
        self._ax.set_ylim(-0.05, 1.10)
        self._ax.grid(True, color="#333344", linewidth=0.5)

        # threshold lines — plain grey, labelled on the right side
        self._ax.axhline(0.20, color="#888888", linewidth=0.7, linestyle=":")
        self._ax.axhline(0.10, color="#888888", linewidth=0.7, linestyle=":")

        # right-side labels for thresholds
        self._warn_label = self._ax.text(
            1.002,
            0.20,
            "WARN (20%)",
            transform=self._ax.get_yaxis_transform(),
            color="#aaaaaa",
            fontsize=7.5,
            verticalalignment="center",
        )
        self._crit_label = self._ax.text(
            1.002,
            0.10,
            "CRITICAL (10%)",
            transform=self._ax.get_yaxis_transform(),
            color="#aaaaaa",
            fontsize=7.5,
            verticalalignment="center",
        )

        # info text bottom-left
        self._info = self._ax.text(
            0.01,
            0.02,
            "",
            transform=self._ax.transAxes,
            color="#aaaaaa",
            fontsize=8,
            verticalalignment="bottom",
            fontfamily="monospace",
        )

        plt.tight_layout()
        # leave a small margin on the right for the threshold labels
        self._fig.subplots_adjust(right=0.85)

    def update(self, _frame) -> None:
        snapshot = self._node.snapshot()
        if not snapshot:
            return

        info_lines = []
        all_x = []

        for vessel, (data, color) in snapshot.items():
            if not data.times:
                continue

            xs = list(data.times)
            ys = list(data.socs)
            all_x.extend(xs)

            # solid line for battery, dashed for generator
            style = "-" if data.active_type == "battery" else "--"

            if vessel not in self._lines:
                (line,) = self._ax.plot(
                    xs, ys, linestyle=style, color=color, linewidth=2, label=vessel
                )
                self._lines[vessel] = line
            else:
                self._lines[vessel].set_data(xs, ys)
                self._lines[vessel].set_linestyle(style)

            icon = "🔋" if data.active_type == "battery" else "⚡"
            info_lines.append(
                f"{icon} {vessel:<22} "
                f"active={data.active_name} ({data.active_type})  "
                f"SOC={ys[-1]:.3f}  "
                f"V={data.voltage:.2f} V"
            )

        if all_x:
            self._ax.set_xlim(max(0.0, min(all_x) - 1), max(all_x) + 2)

        self._info.set_text("\n".join(info_lines))
        self._ax.legend(
            loc="upper right",
            facecolor="#2a2a3e",
            edgecolor="#444444",
            labelcolor="white",
            fontsize=9,
        )
        self._fig.canvas.draw_idle()

    def start(self, interval_ms: int) -> None:
        self._anim = animation.FuncAnimation(
            self._fig, self.update, interval=interval_ms, cache_frame_data=False
        )
        plt.show()


# ── main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="LOTUSim power monitor")
    parser.add_argument(
        "--max-history",
        type=int,
        default=150,
        help="Data points to keep per vessel (default 150 ≈ 5 min at 2 s intervals)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PowerMonitorNode(max_history=args.max_history)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def _refresh_loop():
        while rclpy.ok():
            node.refresh_subscriptions()
            time.sleep(3.0)

    threading.Thread(target=_refresh_loop, daemon=True).start()

    plot = PowerMonitorPlot(node)
    plot.start(interval_ms=500)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
