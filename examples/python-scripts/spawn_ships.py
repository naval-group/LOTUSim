import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random

from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import VesselPositionArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmd, MASCmdArray

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0
OFFSET = 0.0001

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_spawn_node')

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
        self.vessel_poses= {}
        #self.random_publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.print_vessel_positions)
        self.vessel_id = 0 # id number for the number of model in the simulation. Used in randomizing location spawned

    def poses_callback(self, msg):
        for vessel_position in msg.vessels:
            lat = vessel_position.geo_point.latitude
            long = vessel_position.geo_point.longitude
            self.vessel_poses[vessel_position.vessel_name] = (lat, long)

    def print_vessel_positions(self):
        if self.vessel_poses:
            self.get_logger().info(str(self.vessel_poses))

    
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
                    <thrusters1>thruster1</thrusters1>
                </thrusters>
            </underwater>
            <surface>
                <connection_type>XDynWebSocket</connection_type>
                <uri>ws://127.0.0.1:12345</uri>
                <thrusters>
                    <thrusters1>thruster1</thrusters1>
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


    def spawn_aerial_drone(self):
        msg = MASCmdMsg()
        msg.cmd_type =  MASCmdMsg.CREATE_CMD
        msg.model_name =  "x500"
        msg.vessel_name =  f"x500_{self.vessel_id}"     

        geo = GeoPoint()   
        geo.latitude = SPAWN_LATITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        geo.longitude = SPAWN_LONGITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        geo.altitude = 10.0
        msg.geo_point = geo
        
        msg.sdf_string = """
        <lotus_param>
            <render_interface>
                <publish_render>true</publish_render>
                <renderer_type_name>lrauv</renderer_type_name>
            </render_interface>
            <physics_engine_interface>
                  <aerial>
                    <connection_type>ROS2</connection_type>
                    <namespace>aerialWorld</namespace>
                  </aerial>
                  <init_state>Aerial</init_state>
            </physics_engine_interface>
        </lotus_param>
        """
        self.vessel_id += 1
        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_circling_ship(self):
        goal_msg = MASCmd.Goal()
       
        msg = MASCmdMsg()
        msg.cmd_type =  MASCmdMsg.CREATE_CMD
        msg.model_name =  "dtmb_hull"
        msg.vessel_name =  f"dtmb_{self.vessel_id}"     

        geo = GeoPoint()   
        geo.latitude = SPAWN_LATITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        geo.longitude = SPAWN_LONGITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        geo.altitude = SPAWN_ALTITUDE
        msg.geo_point = geo
        
        msg.sdf_string = """
        <lotus_param>
            <waypoint_follower>
                <follower>
                    <loop>true</loop>
                    <linear_accel_limit>0.5</linear_accel_limit>
                    <angular_accel_limit>0.005</angular_accel_limit>
                    <angular_velocities_limits>0.01</angular_velocities_limits>
                    <range_tolerance>2</range_tolerance>
                    <circle>
                        <radius>10</radius>
                    </circle>
                </follower>
            </waypoint_follower>
        </lotus_param>
        """
        self.vessel_id += 1
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_multiple_circling_ship(self,number_of_ships):
        goal_msg = MASCmdArray.Goal()

        for i in range(number_of_ships):
            msg = MASCmdMsg()
            msg.cmd_type =  MASCmdMsg.CREATE_CMD
            msg.model_name =  "dtmb_hull"
            msg.vessel_name =  f"dtmb_{self.vessel_id}"

            geo = GeoPoint()   
            offset = 0.0001
            geo.latitude = SPAWN_LATITUDE + OFFSET * self.vessel_id * random.choice([-1, 1]) 
            geo.longitude = SPAWN_LONGITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
            geo.altitude = SPAWN_ALTITUDE
            msg.geo_point = geo
            
            msg.sdf_string = """
            <lotus_param>
                <waypoint_follower>
                    <follower>
                        <loop>true</loop>
                        <linear_accel_limit>0.5</linear_accel_limit>
                        <angular_accel_limit>0.005</angular_accel_limit>
                        <angular_velocities_limits>0.01</angular_velocities_limits>
                        <range_tolerance>2</range_tolerance>
                        <circle>
                            <radius>100</radius>
                        </circle>
                    </follower>
                </waypoint_follower>
            </lotus_param>
            """
            goal_msg.cmd.append(msg)
            self.vessel_id += 1
        self.mas_array_action_client.wait_for_server()
        return self.mas_array_action_client.send_goal_async(goal_msg)


def main(args=None):
    
    rclpy.init(args=None)

    node = ExampleNode()

    # Spawn 1 cyrcling ship
    future = node.spawn_multiple_circling_ship(1)
    
    rclpy.spin_until_future_complete(node,future)
    node.get_logger().info("Spawn request sent successfully")

    # --- Other examples: uncomment to try ---
    # future2 = node.spawn_ship_with_dynamics(1)
    # rclpy.spin_until_future_complete(node, future2)

    # future3 = node.spawn_aerial_drone(1)
    # rclpy.spin_until_future_complete(node, future3)

    # future4 = node.spawn_circling_ship(2)
    # rclpy.spin_until_future_complete(node, future4)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()