import json
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import VesselCmd, VesselCmdArray, VesselPositionArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmd, MASCmdArray

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
# Domain thresholds in getNewState are z <= -10.0 (Underwater) and z >= 10.0 (Aerial); in between is Surface.
SPAWN_ALTITUDE = -1.0


class ClimbTestNode(Node):
    def __init__(self):
        super().__init__('climb_test_node')
        self.pose_subscription = self.create_subscription(
            VesselPositionArray,
            '/lotusim/poses',
            self.poses_callback,
            10
        )
        self.vessel_poses = {}
        self.last_pose_time = {}
        self.spawned_vessels = []

        self.cmd_publisher = self.create_publisher(
            VesselCmdArray,
            '/lotusim/vessel_cmd_array',
            10
        )
        self.mas_action_client = ActionClient(self, MASCmd, '/lotusim/mas_cmd')

        self.rpm_publisher = {}
        self.position_timer = self.create_timer(1.0, self.print_vessel_positions)
        self.vessel_id = 0

    def poses_callback(self, msg):
        now = time.time()
        for vessel_position in msg.vessels:
            name = vessel_position.vessel_name
            lat = vessel_position.geo_point.latitude
            lon = vessel_position.geo_point.longitude
            alt = vessel_position.geo_point.altitude
            self.vessel_poses[name] = (lat, lon, alt)
            self.last_pose_time[name] = now

    def print_vessel_positions(self):
        if not self.vessel_poses:
            self.get_logger().info("No vessel positions yet")
            return
        now = time.time()
        for name, (lat, lon, alt) in self.vessel_poses.items():
            staleness = now - self.last_pose_time.get(name, now)
            flag = " <- Stale, check for timeout in logs" if staleness > 2.0 else ""
            self.get_logger().info(
                f"{name}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f} "
                f"(last update {staleness:.1f}s ago){flag}"
            )

    def spawn_ship_with_dynamics(self, vessel_name: str):
        msg = MASCmdMsg()
        msg.cmd_type = MASCmdMsg.CREATE_CMD
        msg.model_name = "lrauv"
        vessel_name = f"lrauv_{self.vessel_id}"
        msg.vessel_name = vessel_name

        geo = GeoPoint()
        geo.latitude = SPAWN_LATITUDE
        geo.longitude = SPAWN_LONGITUDE
        geo.altitude = SPAWN_ALTITUDE
        msg.geo_point = geo

        msg.sdf_string = """
        <lotus_param>
            <render_interface>
                <publish_render>true</publish_render>
                <renderer_type_name>lrauv</renderer_type_name>
            </render_interface>
            <physics_engine_interface>
            <underwater>
                <interface_type>XDynWebSocket</interface_type>
                <uri>ws://127.0.0.1:12346</uri>
                <thrusters>
                    <thrusters1>propeller</thrusters1>
                </thrusters>
            </underwater>
            <surface>
                <interface_type>XDynWebSocket</interface_type>
                <uri>ws://127.0.0.1:12345</uri>
                <thrusters>
                    <thrusters1>propeller</thrusters1>
                </thrusters>
            </surface>
            <init_state>Surface</init_state>
            </physics_engine_interface>
        </lotus_param>
        """

        self.rpm_publisher[vessel_name] = self.create_publisher(
            Float64,
            f'/{vessel_name}/rpm',
            10)
        self.get_logger().info(f"Created RPM publisher on /{vessel_name}/rpm")
        self.vessel_id += 1
        self.spawned_vessels.append(vessel_name)

        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def delete_all_vessels(self):
        if not self.spawned_vessels:
            return
        if not self.mas_array_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Action server not available for cleanup")
            return
        goal_msg = MASCmdArray.Goal()
        for name in self.spawned_vessels:
            msg = MASCmdMsg()
            msg.cmd_type = MASCmdMsg.DELETE_CMD
            msg.vessel_name = name
            goal_msg.cmd.append(msg)

        time.sleep(0.2)

        future = self.mas_array_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Delete goal rejected")
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("All vessels deleted")
        self.spawned_vessels.clear()

    def send_propeller_command(self, vessel_name: str, rpm: float, pd: float):
        cmd_array = VesselCmdArray()
        cmd = VesselCmd()
        cmd.vessel_name = vessel_name
        cmd.cmd_string = json.dumps({"propeller(rpm)": rpm, "propeller(P/D)": pd})
        cmd_array.cmds.append(cmd)
        self.cmd_publisher.publish(cmd_array)

        if vessel_name in self.rpm_publisher:
            rpm_msg = Float64()
            rpm_msg.data = rpm
            self.rpm_publisher[vessel_name].publish(rpm_msg)

        self.get_logger().info(
            f"{vessel_name} - propeller command: rpm={rpm}, P/D={pd}")


def main(args=None):
    rclpy.init(args=args)
    node = ClimbTestNode()

    vessel_name = "lrauv_0"
    future = node.spawn_ship_with_dynamics(vessel_name)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    vessel_name = result_future.result().result.name
    node.get_logger().info(f"Spawn request sent successfully, vessel name: {vessel_name}")

    time.sleep(1.0)

    # Positive pitch (P/D) + steady rpm to climb from -12.0 up through the
    # -10.0 Underwater/Surface boundary. Adjust rpm/pd if it doesn't climb
    # fast enough (or climbs too fast to observe) in your build.
    def send_timer_callback():
        node.send_propeller_command(vessel_name, 200.0, 0.88)

    node.create_timer(0.5, send_timer_callback)
    rclpy.spin_once(node)

    shutdown_requested = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    node.get_logger().info(
        f"Watching for domain transition around z=-10.0 (spawned at {SPAWN_ALTITUDE}). "
        "Watch this log for 'STALE' and the LOTUSim terminal for "
        "'websocket timed out' after the transition."
    )

    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.get_logger().info("Shutting down, deleting vessels...")
        node.delete_all_vessels()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()