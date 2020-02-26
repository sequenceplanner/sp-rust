import sys
import os
import rclpy
import time

from builtin_interfaces.msg import Time

from rclpy.node import Node
from cubes_1_msgs.msg import SMCommand
from cubes_1_msgs.msg import SMState
from visualization_msgs.msg import Marker
from ros2_scene_manipulation_msgs.srv import ManipulateScene

class Cubes1SceneMaster(Node):

    def __init__(self):
        super().__init__("cubes_1_scene_master")

        self.manipulate_scene_client = self.create_client(ManipulateScene, 'scene_manipulation_service')

        while not self.manipulate_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ManipulateScene.Request()

        self.callback_timeout = time.time()
        self.timeout = 5

        self.sm_state_timer_period = 0.2
        self.marker_timer_period = 0.01

        self.sm_command = SMCommand()
        self.sm_command.attach_r1_box = False                       
        self.sm_command.attach_r2_box = False

        self.tick_inhibited = False
        self.prev_sm_command = SMCommand()
        self.prev_sm_command.attach_r1_box = False                       
        self.prev_sm_command.attach_r2_box = False    

        self.viz_marker_publisher_ = self.create_publisher(
            Marker,
            "visualization_marker",
            10)             

        self.sm_command_subscriber = self.create_subscription(
            SMCommand, 
            "cubes/sm/command",
            self.sp_command_callback,
            10)
        
        time.sleep(2)

        self.sm_state = SMState()
        self.sm_state.attached_r1_box = False
        self.sm_state.attached_r2_box = False
        self.sm_state.echo = self.sm_command

        self.sm_state_publisher_ = self.create_publisher(
            SMState,
            "cubes/sm/state",
            10)

        self.sm_state_timer = self.create_timer(
            self.sm_state_timer_period, 
            self.sm_state_publisher_callback)

        self.marker_timer = self.create_timer(
            self.marker_timer_period, 
            self.load_markers)

    def load_markers(self):
        # clock = Clock()
        self.marker = Marker()
        self.marker.header.frame_id = "cubes/c1/cube"
        self.marker.header.stamp = Time()
        self.marker.ns = ""
        self.marker.id = 0
        self.marker.type = 1
        self.marker.action = 0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.viz_marker_publisher_.publish(self.marker);
    
    def sm_state_publisher_callback(self):
        if time.time() < self.callback_timeout:
            self.sm_state_publisher_.publish(self.sm_state)
        else:
            self.sm_state.echo.attach_r1_box = self.sm_state.attached_r1_box
            self.sm_state.echo.attach_r2_box = self.sm_state.attached_r2_box
            self.sm_state_publisher_.publish(self.sm_state)
     
    def inhibit_tick(self):
        self.prev_sm_command.attach_r1_box = self.sm_command.attach_r1_box
        self.prev_sm_command.attach_r2_box = self.sm_command.attach_r2_box

    def sp_command_callback(self, data):

        self.callback_timeout = time.time() + self.timeout

        self.sm_command.attach_r1_box = data.attach_r1_box
        self.sm_command.attach_r2_box = data.attach_r2_box

        # if self.sm_command.attach_r1_box == self.prev_sm_command.attach_r1_box and \
        #    self.sm_command.attach_r1_box == self.prev_sm_command.attach_r1_box:
        #     # print("NOT CHANGING STUFF")
        #     self.tick_inhibited = True
        # else:
        #     # print("CHANGING STUFF")
        #     self.tick_inhibited = False
        
        # if self.tick_inhibited == False:
            # self.inhibit_tick()
        if self.sm_command.attach_r1_box == True:
            self.fn_attach_r1_box()
        elif self.sm_command.attach_r2_box == True:
            self.fn_attach_r2_box()
        elif self.sm_command.attach_r1_box == False and self.sm_command.attach_r2_box == False:
            self.fn_detach_box()
        else:
            pass
        # else:
        #     pass   

    def send_request(self, frame, parent, pos):
        self.req.frame_id = frame
        self.req.parent_id = parent
        # self.req.transform.translation.x = 0.0
        # self.req.transform.translation.y = 0.0
        # self.req.transform.translation.z = 0.0
        # self.req.transform.rotation.x = 0.0
        # self.req.transform.rotation.y = 0.0
        # self.req.transform.rotation.z = 0.0
        # self.req.transform.rotation.w = 0.0
        self.req.same_position_in_world = pos
        self.future = self.manipulate_scene_client.call_async(self.req)

    def fn_attach_r1_box(self):
        print("ATTACH TO R1")
        self.send_request("cubes/c1/cube", "cubes/r1/meca_axis_1_link", True)
        self.sm_state.attached_r1_box = True
        self.sm_state.attached_r2_box = False
    
    def fn_attach_r2_box(self):
        print("ATTACH TO R2")
        self.send_request("cubes/c1/cube", "cubes/r2/meca_axis_1_link", True)
        self.sm_state.attached_r2_box = True
        self.sm_state.attached_r1_box = False
    
    def fn_detach_box(self):
        print("DETACH")
        self.send_request("cubes/c1/cube", "world", True)     
        self.sm_state.attached_r1_box = False
        self.sm_state.attached_r2_box = False 
       
def main(args=None):
    rclpy.init(args=args)
    
    cubes_master = Cubes1SceneMaster()
    rclpy.spin(cubes_master)
    cubes_master.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
