import sys
import rclpy
import time
import csv
import os
import math
import numpy
import ast
import json
import yaml
import pyfirmata
from rclpy.node import Node

from control_box_msgs.msg import Goal
from control_box_msgs.msg import State

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode
from ament_index_python.packages import get_package_share_directory

class ControlBoxDriver(Node):

    def __init__(self):
        super().__init__("control_box_driver")

        self.board = pyfirmata.Arduino('/dev/ttyARDUINO')

        # internal state
        self.blue_light = 0

        # initial state off
        self.board.digital[3].write(1)

        # remember last goal
        self.last_seen_goal = Goal()
        self.last_seen_goal.blue_light = self.blue_light

        # sp node mode
        self.sp_node_cmd = NodeCmd()
        self.mode = NodeMode()
        self.mode.mode = "init"

        self.subscriber = self.create_subscription(
            Goal,
            "/control_box/goal",
            self.sp_goal_callback,
            10)

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "/control_box/node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.state_publisher = self.create_publisher(
            State,
            "/control_box/state",
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "/control_box/mode",
            10)

        print('Up and running...')


    def sp_goal_callback(self, data):
        if self.last_seen_goal == data:
            return

        self.last_seen_goal = data
        self.blue_light = data.blue_light
        self.get_logger().info('goal: "%s"' % data)

        # update light
        self.board.digital[3].write(self.blue_light == False)
        self.get_logger().info("light is " + ('on' if self.blue_light else 'off'))

        msg = State()
        msg.blue_light_on = self.blue_light
        self.state_publisher.publish(msg)


    def sp_node_cmd_callback(self, data):
        self.get_logger().info('"%s"' % data)
        self.sp_node_cmd = data

        # move to general function in sp
        echo_msg = {}
        for k in Goal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(self.last_seen_goal, "_"+k)})

        self.mode.echo = json.dumps(echo_msg)

        if self.sp_node_cmd.mode == "run":
            self.mode.mode = "running"
        else:
            self.mode.mode = "init"

        self.sp_mode_publisher.publish(self.mode)
        # because this node is not ticking, we also publish our
        # measured states here
        msg = State()
        msg.blue_light_on = self.blue_light
        self.state_publisher.publish(msg)





def main(args=None):

    rclpy.init(args=args)
    node = ControlBoxDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
