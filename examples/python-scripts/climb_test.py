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

# positive climbs, negative sinks
CLIMB_THRUST = 8.0
DIVE_THRUST = -8.0
# how often to flip direction
FLIP_PERIOD_SEC = 4.0
SPAWN_Z = -10.0

class OscillateTestNode(Node):
    def __init__(self):
        super().__init__('oscillate_test_node')
        self.pose_subscription = self.create_subscription(VesselPositionArray, '/lotusim/poses', self.poses_callback, 10)
        self.vessel_poses = {}
        self.last_pose_time = {}
        self.spawned_vessels = []

        self.cmd_publisher = self.create_publisher(VesselCmdArray, '/lotusim/vessel_cmd_array', 10)
        self.mas_action_client = ActionClient(self, MASCmd, '/lotusim/mas_cmd')

        self.rpm_publisher = {}
        self.position_timer = self.create_timer(1.0, self.print_vessel_positions)
        self.vessel_id = 0

        self.current_thrust = CLIMB_THRUST
        self.direction_timer = self.create_timer(FLIP_PERIOD_SEC, self.flip_direction)

    def flip_direction(self):
        self.current_thrust = (
            DIVE_THRUST if self.current_thrust == CLIMB_THRUST else CLIMB_THRUST
        )
        label = "DIVING" if self.current_thrust < 0 else "CLIMBING"
        self.get_logger().info(f"=== Flipping direction: now {label} (thrust={self.current_thrust}) ===")

    def poses_callback(self, msg):
        now = time.time()
        for vessel_position in msg.vessels:
            name = vessel_position.vessel_name
            z = vessel_position.pose.position.z
            self.vessel_poses[name] = z
            self.last_pose_time[name] = now

    def print_vessel_positions(self):
        if not self.vessel_poses:
            self.get_logger().info("No vessel positions yet")
            return
        now = time.time()
        for name, z in self.vessel_poses.items():
            staleness = now - self.last_pose_time.get(name, now)
            flag = " <- STALE, check for timeout in logs" if staleness > 3.0 else ""
            self.get_logger().info(
                f"{name}: z={z:.2f} (last update {staleness:.1f}s ago){flag}"
            )

    def spawn_ship_with_dynamics(self, vessel_name: str):
        msg = MASCmdMsg()
        msg.cmd_type = MASCmdMsg.CREATE_CMD
        msg.model_name = "bluerov2_heavy"
        vessel_name = f"bluerov2_heavy{self.vessel_id}"
        msg.vessel_name = vessel_name

        msg.vessel_position.position.x = 0.0
        msg.vessel_position.position.y = 0.0
        msg.vessel_position.position.z = SPAWN_Z
        msg.vessel_position.orientation.w = 1.0 

        msg.sdf_string = """
        <lotus_param>
            <render_interface>
                <publish_render>true</publish_render>
                <renderer_type_name>bluerov2_heavy</renderer_type_name>
            </render_interface>
            <physics_engine_interface>
            <underwater>
                <interface_type>XDynWebSocket</interface_type>
                <uri>ws://127.0.0.1:12346</uri>
                <thrusters>
                    <thrusters1>bluerov2_heavy_prop_1</thrusters1>
                    <thrusters2>bluerov2_heavy_prop_2</thrusters2>
                    <thrusters3>bluerov2_heavy_prop_3</thrusters3>
                    <thrusters4>bluerov2_heavy_prop_4</thrusters4>
                    <thrusters5>bluerov2_heavy_prop_5</thrusters5>
                    <thrusters6>bluerov2_heavy_prop_8</thrusters6>
                </thrusters>
            </underwater>
            <surface>
                <interface_type>XDynWebSocket</interface_type>
                <uri>ws://127.0.0.1:12345</uri>
                <thrusters>
                    <thrusters1>bluerov2_heavy_prop_1</thrusters1>
                    <thrusters2>bluerov2_heavy_prop_2</thrusters2>
                    <thrusters3>bluerov2_heavy_prop_3</thrusters3>
                    <thrusters4>bluerov2_heavy_prop_4</thrusters4>
                    <thrusters5>bluerov2_heavy_prop_5</thrusters5>
                    <thrusters6>bluerov2_heavy_prop_8</thrusters6>
                </thrusters>
            </surface>
            <init_state>Underwater</init_state>
            </physics_engine_interface>
        </lotus_param>
        """

        self.rpm_publisher[vessel_name] = self.create_publisher(Float64, f'/{vessel_name}/rpm', 10)
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

    def send_dive_command(self, vessel_name: str, thrust: float):
        cmd_array = VesselCmdArray()
        cmd = VesselCmd()
        cmd.vessel_name = vessel_name
        cmd.cmd_string = json.dumps({
            "bluerov2_heavy_prop_1(T)": 0.0,
            "bluerov2_heavy_prop_2(T)": 0.0,
            "bluerov2_heavy_prop_3(T)": 0.0,
            "bluerov2_heavy_prop_4(T)": 0.0,
            "bluerov2_heavy_prop_5(T)": thrust,
            "bluerov2_heavy_prop_8(T)": thrust,
        })
        cmd_array.cmds.append(cmd)
        self.cmd_publisher.publish(cmd_array)


def main(args=None):
    rclpy.init(args=args)
    node = OscillateTestNode()

    vessel_name = "bluerov2_heavy_0"
    future = node.spawn_ship_with_dynamics(vessel_name)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    vessel_name = result_future.result().result.name

    time.sleep(1.0)

    def send_timer_callback():
        node.send_dive_command(vessel_name, node.current_thrust)

    node.create_timer(0.2, send_timer_callback)
    rclpy.spin_once(node)

    shutdown_requested = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

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