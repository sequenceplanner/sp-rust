import sys
import os
import rclpy
import time
import json

from builtin_interfaces.msg import Time

from rclpy.node import Node
from cubes_1_msgs.msg import SMCommand
from sp_messages.msg import RunnerInfo
from sp_messages.msg import State
from cubes_1_msgs.msg import SMState
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
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

        self.sp_path_to_product_name = {
            'cubes/buffer1_holding': 'buffer1',
            'cubes/buffer2_holding': 'buffer2',
            'cubes/r1_holding': 'robot1',
            'cubes/r2_holding': 'robot2',
            'cubes/table_holding': 'table1',
            'cubes/table2_holding': 'table2',
        }

        self.products = {
            'robot1' : 0,
            'robot2' : 0,
            'buffer1' : 0,
            'buffer2' : 0,
            'table1' : 0,
            'table2' : 0,
        }

        self.product_to_frame = {
            1: 'cubes/c1/cube',
            2: 'cubes/c2/cube',
            3: 'cubes/c3/cube',
        }

        red = ColorRGBA()
        red.a = 1.0
        red.r = 1.0
        green = ColorRGBA()
        green.a = 1.0
        green.g = 1.0
        blue = ColorRGBA()
        blue.a = 1.0
        blue.b = 1.0
        self.product_colors = {
            1: red,
            2: green,
            3: blue,
        }

        self.slot_to_frame = {
            'robot1' : 'cubes/r1/meca_axis_6_link',
            'robot2' : 'cubes/r2/meca_axis_6_link',
            'buffer1' : 'cubes/buffer1',
            'buffer2' : 'cubes/buffer2',
            'table1' : 'cubes/table1',
            'table2' : 'cubes/table2',
        }

        self.product_marker_publishers = {
            1: self.create_publisher(Marker, "cubes/sm/cube1_marker", 20),
            2: self.create_publisher(Marker, "cubes/sm/cube2_marker", 20),
            3: self.create_publisher(Marker, "cubes/sm/cube3_marker", 20),
        }

        self.sm_sp_runner_subscriber = self.create_subscription(
            RunnerInfo,
            "sp/runner/info",
            self.sp_runner_callback,
            20)

        self.marker_timer = self.create_timer(
            self.marker_timer_period,
            self.publish_markers)

    def cube_marker(self, frame, color):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = Time()
        marker.ns = ""
        marker.id = 0
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color

        return marker

    def publish_markers(self):
        for key, frame in self.product_to_frame.items():
            color = self.product_colors[key]
            marker = self.cube_marker(frame, color)
            self.product_marker_publishers[key].publish(marker)

    def sp_runner_callback(self, data):
        old = set(self.products.items())
        for s in data.state:
            pn = self.sp_path_to_product_name.get(s.path)
            if pn != None:
                v = json.loads(s.value_as_json)
                self.products[pn] = v

        # update what has changed
        new = set(self.products.items())
        changes = new - old
        if changes != set():
            print(changes)
            for slot, prod in changes:
                self.handle_change(slot, prod)


    def handle_change(self, slot, prod):
        for key, value in self.products.items():
            if value == prod:
                self.products[key] = 0  # "move" the item (it can only be in one slot)
        self.products[slot] = prod
        # we never "detach" an item, just attach it to its new owner. (with teleport)
        if prod != 0:
            child = self.product_to_frame[prod]
            parent = self.slot_to_frame[slot]
            self.send_request(child, parent, False)

    def send_request(self, frame, parent, pos):
        self.req.frame_id = frame
        self.req.parent_id = parent
        self.req.transform.translation.x = 0.0
        self.req.transform.translation.y = 0.0
        self.req.transform.translation.z = 0.0
        self.req.transform.rotation.x = 0.0
        self.req.transform.rotation.y = 0.0
        self.req.transform.rotation.z = 0.0
        self.req.transform.rotation.w = 1.0
        self.req.same_position_in_world = pos
        self.future = self.manipulate_scene_client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    cubes_master = Cubes1SceneMaster()
    rclpy.spin(cubes_master)
    cubes_master.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
