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
from rclpy.node import Node

from operator_ui_msgs.msg import Goal
from operator_ui_msgs.msg import State

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode
from ament_index_python.packages import get_package_share_directory

class OperatorUIDriver(Node):

    def __init__(self):
        super().__init__("operator_ui_driver")

        # internal state
        self.instruction = "Nothing to do"
        self.current_state = ""

        # remember last goal
        self.last_seen_goal = Goal()

        # sp node mode
        self.sp_node_cmd = NodeCmd()
        self.mode = NodeMode()
        self.mode.mode = "init"

        self.subscriber = self.create_subscription(
            Goal,
            "/operator_ui/goal",
            self.sp_cmd_callback,
            10)

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "/operator_ui/node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.state_publisher = self.create_publisher(
            State,
            "/operator_ui/state",
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "/operator_ui/mode",
            10)

        print('Up and running...')


    def sp_cmd_callback(self, data):
        self.last_seen_goal = data
        self.instruction = data.instruction
        self.get_logger().info('goal: "%s"' % data)

        msg = State()
        msg.state = self.current_state
        self.state_publisher.publish(msg)


    def sp_node_cmd_callback(self, data):
        self.get_logger().info('"%s"' % data)
        self.node_cmd = data

        # move to general function in sp
        echo_msg = {}
        for k in Goal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(self.last_seen_goal, "_"+k)})

        self.mode.echo = json.dumps(echo_msg)

        if self.node_cmd.mode == "run":
            self.mode.mode = "running"
        else:
            self.mode.mode = "init"

        self.sp_mode_publisher.publish(self.mode)





def main(args=None):

    rclpy.init(args=args)
    node = OperatorUIDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
