import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random
import signal
import time

from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import VesselPositionArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmd, MASCmdArray
from lotusim_msgs.srv import SetWaypoints

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0
OFFSET = 0.0001

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_spawn_node', namespace='/lotusim')

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
        self.spawned_vessels = []  # track spawned vessels for cleanup
        self.waypoint_client = {}
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
        vessel_name =  f"lrauv_{self.vessel_id}"     
        msg.vessel_name = vessel_name

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
        self.spawned_vessels.append(vessel_name)

        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_aerial_drone(self):
        msg = MASCmdMsg()
        msg.cmd_type =  MASCmdMsg.CREATE_CMD
        msg.model_name =  "x500"
        vessel_name =  f"x500_{self.vessel_id}"     
        msg.vessel_name = vessel_name

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
        self.spawned_vessels.append(vessel_name)

        goal_msg = MASCmd.Goal()
        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_circling_ship(self):
        goal_msg = MASCmd.Goal()
       
        msg = MASCmdMsg()
        msg.cmd_type =  MASCmdMsg.CREATE_CMD
        msg.model_name =  "dtmb_hull"
        vessel_name =  f"dtmb_{self.vessel_id}"     
        msg.vessel_name = vessel_name

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
        self.spawned_vessels.append(vessel_name)

        goal_msg.cmd = msg
        self.mas_action_client.wait_for_server()
        return self.mas_action_client.send_goal_async(goal_msg)

    def spawn_multiple_circling_ship(self,number_of_ships):
        goal_msg = MASCmdArray.Goal()

        for i in range(number_of_ships):
            msg = MASCmdMsg()
            msg.cmd_type =  MASCmdMsg.CREATE_CMD
            msg.model_name =  "dtmb_hull"
            vessel_name =  f"dtmb_{self.vessel_id}"
            msg.vessel_name = vessel_name

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
            self.spawned_vessels.append(vessel_name)
        self.mas_array_action_client.wait_for_server()
        return self.mas_array_action_client.send_goal_async(goal_msg)
    
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

        # Small delay to let Ctrl+C turbulence settle
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

    def setupRosForModel(self, model_name:str) :
        if not model_name in self.waypoint_client:
            client = self.create_client(SetWaypoints, model_name + '/waypoints')
            for attempt in range(1, 4):
                if client.wait_for_service(timeout_sec=1.0):
                    self.waypoint_client[model_name] = client
                    return
                print(f'Waiting for waypoint service... (attempt {attempt}/3)')
            print(f'Failed to connect to waypoint service for {model_name}')
        
    def send_random_waypoint_request(self, model_name:str):
        if not model_name in self.waypoint_client:
            try:
                self.setupRosForModel(model_name)
            except ValueError as e:
                print("Sending waypoint failed. Try again.")
                return 
        request = SetWaypoints.Request()

        latitude  = SPAWN_LATITUDE  + OFFSET * self.vessel_id * random.choice([-1, 1])
        longitude = SPAWN_LONGITUDE + OFFSET * self.vessel_id * random.choice([-1, 1])
        request.path = [GeoPoint(latitude=latitude, longitude=longitude, altitude=SPAWN_ALTITUDE)]

        request.loop = False

        future = self.waypoint_client[model_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            print(f'Waypoint set: ({latitude:.7f}, {longitude:.7f})')
        else:
            print('Failed to set waypoints')

def main(args=None):
    
    rclpy.init(args=None)

    node = ExampleNode()

    # Spawn 1 cyrcling ship
    future = node.spawn_multiple_circling_ship(1)
    rclpy.spin_until_future_complete(node,future)
    node.get_logger().info("Spawn request sent successfully")

    # Send a random waypoint to the ship
    node.send_random_waypoint_request("dtmb_0")

    # --- Other examples: uncomment to try ---
    # future2 = node.spawn_ship_with_dynamics(1)
    # rclpy.spin_until_future_complete(node, future2)

    # future3 = node.spawn_aerial_drone(1)
    # rclpy.spin_until_future_complete(node, future3)

    # future4 = node.spawn_circling_ship(2)
    # rclpy.spin_until_future_complete(node, future4)

    shutdown_requested = False
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    while rclpy.ok() and not shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    # ROS still up here — cleanup works
    node.get_logger().info("Shutting down, deleting vessels...")
    node.delete_all_vessels()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()