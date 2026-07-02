import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import VesselPositionArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmd, MASCmdArray
import math

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0

class RadarTestSpawner(Node):
    def __init__(self):
        super().__init__('demo_test_spawner')

        self.pose_subscription = self.create_subscription(
            VesselPositionArray,
            '/lotusim/poses',
            self.poses_callback,
            10)
        
        self.mas_action_client = ActionClient(
            self,
            MASCmd,
            '/lotusim/mas_cmd'
        )
        self.mas_array_action_client = ActionClient(
            self,
            MASCmdArray,
            '/lotusim/mas_cmd_array'
        )
        self.vessel_poses = {}
        self.vessel_id = 0

    def poses_callback(self, msg):
        for vessel_position in msg.vessels:
            lat = vessel_position.geo_point.latitude
            lon = vessel_position.geo_point.longitude
            self.vessel_poses[vessel_position.vessel_name] = (lat, lon)

    def spawn_static_dtmb(self):
        """Spawn a single dtmb ship at the center"""
        msg = MASCmdMsg()
        msg.cmd_type = MASCmdMsg.CREATE_CMD
        
        msg.model_name = "dtmb_hull"  
        msg.vessel_name = "dtmb_0" 

        geo = GeoPoint()
        geo.latitude = SPAWN_LATITUDE
        geo.longitude = SPAWN_LONGITUDE
        geo.altitude = SPAWN_ALTITUDE
        msg.geo_point = geo

        msg.sdf_string = """
        <lotus_param>
            <waypoint_follower>
                <follower>
                    <loop>false</loop>
                </follower>
            </waypoint_follower>
        </lotus_param>
        """

        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_fremm_circle(self, n_ships=5, radius_m=60):
        """Spawn n_ships fremm in a circle around the DTMB"""
        goal_msg = MASCmdArray.Goal()

        for i in range(n_ships):
            angle = (2 * math.pi / n_ships) * i
            lat_offset = (radius_m / 111111.0) * math.cos(angle)  # approx conversion: 1 deg lat ~ 111km
            lon_offset = (radius_m / (111111.0 * math.cos(math.radians(SPAWN_LATITUDE)))) * math.sin(angle)

            msg = MASCmdMsg()
            msg.cmd_type = MASCmdMsg.CREATE_CMD
            msg.model_name = "fremm"
            msg.vessel_name = f"fremm_{i}"

            geo = GeoPoint()
            geo.latitude = SPAWN_LATITUDE + lat_offset
            geo.longitude = SPAWN_LONGITUDE + lon_offset
            geo.altitude = SPAWN_ALTITUDE
            msg.geo_point = geo

            # Static LRAUV (no physics)
            msg.sdf_string = """
            <lotus_param>
                <render_interface>
                    <publish_render>false</publish_render>
                </render_interface>
            </lotus_param>
            """
            goal_msg.cmd.append(msg)

        self.mas_array_action_client.wait_for_server()
        return self.mas_array_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RadarTestSpawner()

    # Spawn central dtmb ship
    future_dtmb = node.spawn_static_dtmb()
    rclpy.spin_until_future_complete(node, future_dtmb)
    node.get_logger().info("DTMB ship spawned")

    # Spawn 10 LRAUVs around it
    future_fremm= node.spawn_fremm_circle(n_ships=5, radius_m=60)
    rclpy.spin_until_future_complete(node, future_fremm)
    node.get_logger().info("5 fremm ships spawned around dtmb")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()