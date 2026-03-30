import json
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import VesselCmd, VesselCmdArray, VesselPositionArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmd

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0
OFFSET = 0.0001

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_control_node')
        self.pose_subscription = self.create_subscription(
            VesselPositionArray,
            '/lotusim/poses',
            self.poses_callback,
            10
        )
        self.vessel_poses= {}
        self.cmd_publisher = self.create_publisher(
            VesselCmdArray,
            '/lotusim/vessel_cmd_array',
            10
        )
        self.mas_action_client = ActionClient(self, MASCmd, '/lotusim/mas_cmd')
        self.position_timer = self.create_timer(1.0, self.print_vessel_positions)
        self.vessel_id = 0 # id number for the number of model in the simulation. Used in randomizing location spawned

    def poses_callback(self, msg):
        for vessel_position in msg.vessels:
            name = vessel_position.vessel_name
            lat = vessel_position.geo_point.latitude
            long = vessel_position.geo_point.longitude
            self.vessel_poses[name] = (lat, long)

    def print_vessel_positions(self):
        if not self.vessel_poses:
            self.get_logger().info("No vessel positions yet")
            return
        for name, (lat, lon) in self.vessel_poses.items():
            self.get_logger().info(f"{name}: latitude={lat:.6f}, longitude={lon:.6f}")

    def spawn_ship_with_dynamics(self):
        msg = MASCmdMsg()
        msg.cmd_type =  MASCmdMsg.CREATE_CMD
        msg.model_name =  "lrauv"
        msg.vessel_name =  f"lrauv_{self.vessel_id}"        

        geo = GeoPoint()   
        geo.latitude = SPAWN_LATITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        geo.longitude = SPAWN_LONGITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
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
                <connection_type>XDynWebSocket</connection_type>
                <uri>ws://127.0.0.1:12346</uri>
                <thrusters>
                    <thrusters1>propeller</thrusters1>
                </thrusters>
            </underwater>
            <surface>
                <connection_type>XDynWebSocket</connection_type>
                <uri>ws://127.0.0.1:12345</uri>
                <thrusters>
                    <thrusters1>propeller</thrusters1>
                </thrusters>
            </surface>
            <init_state>Underwater</init_state>
            </physics_engine_interface>
        </lotus_param>
        """
        self.vessel_id += 1
        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def send_propeller_command(self, vessel_name:str, propeller_name: str, rpm: float, pd: float):
        """
        Prepare and send a propeller command message.

        Args:
            rpm (float): Propeller rotation speed in RPM.
            pd (float): Propeller pitch-to-diameter ratio.
        """

        # Create a VesselCmdArray message
        cmd_array = VesselCmdArray()
        cmd = VesselCmd()
        cmd.vessel_name = vessel_name 
        cmd.cmd_string = json.dumps({"propeller(rpm)": rpm, "propeller(P/D)": pd})
        cmd_array.cmds.append(cmd)

        # Publish the message
        self.cmd_publisher.publish(cmd_array)

        # Also log with ROS logger
        print(f"{cmd.vessel_name} - propeller command published: rpm={rpm}, P/D={pd}")
        return True

def main(args=None):
    rclpy.init(args=args)

    node = ExampleNode()
    
    future = node.spawn_ship_with_dynamics()
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("Spawn request sent successfully")

    time.sleep(1.0)
    
    def send_timer_callback():
        node.send_propeller_command("lrauv_1", "propeller", 200, 0.88)

    node.create_timer(0.5, send_timer_callback)
    
    rclpy.spin_once(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()