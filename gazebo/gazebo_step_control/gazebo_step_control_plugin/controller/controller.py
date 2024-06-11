import sys
from threading import Thread
from multiprocessing import shared_memory
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from gazebo_step_control_interface.srv import StepControl
from multiprocessing.resource_tracker import unregister
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
import time

class SteppingController(Node):

    def __init__(self):
        super().__init__('stepping_client')
        self.step_enable_service = self.create_client(SetBool, 'step_control_enable')
        while not self.step_enable_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('step_control_enable service not available, waiting again...')
        
        self.step_control_service = self.create_client(StepControl, 'step')
        while not self.step_control_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('step control service not available, waiting again...')
        #self.existing_shm = None
        self.existing_shm = shared_memory.SharedMemory(name='physics2')

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            .01, self.timer_callback, callback_group=self.timer_cb_group)
        
    def timer_callback(self):
        try:
            self.timer.cancel()
            self.step()
            self.timer.reset()
        except Exception as e:
            self.get_logger().error(f"Exception occured: {e}")
        pass

    def __del__(self):        
        self.existing_shm.unlink()
        print("Deleting")
        # body of destructor
    
    def step(self):
        step_req = StepControl.Request()
        step_req.steps = 1
        step_req.block = False       
        self.step_control_service.call(step_req)
        if self.existing_shm is not None:
            print(self.existing_shm.buf[0])
        
    def run(self):
        enable_req = SetBool.Request()
        enable_req.data = True        
        self.step_enable_service.call(enable_req)
        while True:
             step_req = StepControl.Request()
             step_req.steps = 1
             step_req.block = True       
             self.step_control_service.call(step_req)
             #if self.existing_shm is not None:
             #   print(self.existing_shm.buf[0])

def main(args=None):
    rclpy.init(args=args)
    stepping_client = SteppingController()
    #spin_thread = Thread(target=rclpy.spin, args=(stepping_client,))
    #spin_thread.start()
    #stepping_client.run()
    #stepping_client.destroy_node()
    #rclpy.shutdown()
    executor = MultiThreadedExecutor()
    executor.add_node(stepping_client)
    executor.spin()
    # exit_code = spawn_entity_node.run()
    

if __name__ == '__main__':
    main()