import sys
import os
import rclpy
import time

from builtin_interfaces.msg import Time

from rclpy.node import Node
from cubes_1_msgs.msg import SMCommand
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

        self.products = {
            'robot1' : 0,
            'robot2' : 0,
            'buffer1' : 0,
            'buffer2' : 0,
            'table1' : 1,
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

        self.sm_command_subscriber = self.create_subscription(
            SMCommand,
            "cubes/sm/command",
            self.sp_command_callback,
            20)

        self.sm_state_publisher_ = self.create_publisher(
            SMState,
            "cubes/sm/state",
            20)

        self.sm_state_timer = self.create_timer(
            self.sm_state_timer_period,
            self.sm_state_publisher_callback)

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

    def sm_state_publisher_callback(self):
        # wanted to send an array but its too messy in sp...
        state = SMState()
        state.robot1 = self.products['robot1']
        state.robot2 = self.products['robot2']
        state.buffer1 = self.products['buffer1']
        state.buffer2 = self.products['buffer2']
        state.table1 = self.products['table1']
        state.table2 = self.products['table2']

        self.sm_state_publisher_.publish(state)

    def sp_command_callback(self, data):
        for key, value in self.products.items():
            if value == data.value:
                self.products[key] = 0  # "move" the item (it can only be in one slot)
        self.products[data.slot_id] = data.value
        # we never "detach" an item, just attach it to its new owner. (with teleport)
        if data.value != 0:
            child = self.product_to_frame[data.value]
            parent = self.slot_to_frame[data.slot_id]
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
