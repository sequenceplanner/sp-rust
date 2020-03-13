import sys
import time
import ast
import csv
import os
import math
import threading
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sp_operator_msgs.msg import Goal
from sp_operator_msgs.msg import State
from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks():
    state = State(doing = "away", e_stop = False)
    goal = Goal(to_do = "ready")
    domain = ["away", "ready", "working", "goal1", "goal2", "goal3"]

    trigger_node = None
    trigger_ui = None

    def __init__(self):
        super(Callbacks, self).__init__()

class OperatorUI(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "sp_operator")
        Callbacks.__init__(self)

        Callbacks.trigger_node = self.trigger

        # sp node mode
        self.sp_node_cmd = NodeCmd()
        self.mode = NodeMode()
        self.mode.mode = "init"

        self.subscriber = self.create_subscription(
            Goal,
            "/sp_operator/goal",
            self.sp_cmd_callback,
            10)

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "/sp_operator/node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.state_publisher = self.create_publisher(
            State,
            "/sp_operator/state",
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "/sp_operator/mode",
            10)

        print('Up and running...')
    
    def trigger(self):
        print("trigger node")
        self.state_publisher.publish(Callbacks.state)
    
    def sp_cmd_callback(self, data):
        Callbacks.goal = data
        self.get_logger().info('goal: "%s"' % data)
        Callbacks.trigger_ui()

        self.state_publisher.publish(Callbacks.state)


    def sp_node_cmd_callback(self, data):
        self.get_logger().info('"%s"' % data)
        self.node_cmd = data
        Callbacks.goal.to_do = data.mode
        Callbacks.trigger_ui()

        # move to general function in sp
        echo_msg = {}
        for k in Goal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(Callbacks.goal, "_"+k)})

        self.mode.echo = json.dumps(echo_msg)

        if self.node_cmd.mode == "run":
            self.mode.mode = "running"
        else:
            self.mode.mode = "init"

        self.sp_mode_publisher.publish(self.mode)


class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        Callbacks.trigger_ui = self.trigger

        self.label_goal = QLabel(Callbacks.goal.to_do)
        self.goal_box_l = QHBoxLayout()
        self.goal_box_l.addWidget(QLabel("To do:"))
        self.goal_box_l.addWidget(self.label_goal)
        self.goal_box = QGroupBox("Goal")
        self.goal_box.setLayout(self.goal_box_l)

        self.label_state_doing = QLabel(Callbacks.state.doing)
        self.label_state_estop = QLabel(str(Callbacks.state.e_stop))
        self.state_box_l = QHBoxLayout()
        self.state_box_l.addWidget(QLabel("Doing:"))
        self.state_box_l.addWidget(self.label_state_doing)
        self.state_box_l.addWidget(QLabel("E-stop:"))
        self.state_box_l.addWidget(self.label_state_estop)
        self.state_box = QGroupBox("State")
        self.state_box.setLayout(self.state_box_l)

        self.line_box_1 = QGroupBox("Doing")
        self.doing_input = QLineEdit()
        self.doing_input.setMaximumWidth(120)
        self.button_1 = QPushButton('set doing')
        self.button_2 = QPushButton('set goal')
        self.line_box_1_layout = QHBoxLayout()
        self.line_box_1_layout.addWidget(self.doing_input)
        self.line_box_1_layout.addWidget(self.button_1)
        self.line_box_1_layout.addWidget(self.button_2)
        self.line_box_1.setLayout(self.line_box_1_layout)

        self.stop_box = QGroupBox("emergency_stop")
        self.stop_box_layout = QHBoxLayout()
        self.stop_button = QPushButton("STOP")
        self.stop_button.setMaximumWidth(80)
        self.stop_box_layout.addWidget(self.stop_button)
        self.stop_box.setLayout(self.stop_box_layout)

        def stop_button_clicked():
            print('EMERGENCY STOP')
            Callbacks.state.e_stop = not Callbacks.state.e_stop
            Callbacks.trigger_node()
            self.trigger_ui()

        self.stop_button.clicked.connect(stop_button_clicked)

        def button_1_clicked():
            Callbacks.state.doing = self.doing_input.text()
            Callbacks.trigger_node()
            self.trigger_ui()

        self.button_1.clicked.connect(button_1_clicked)

        def button_2_clicked():
            Callbacks.state.doing = Callbacks.goal.to_do
            Callbacks.trigger_node()
            self.trigger_ui()

        self.button_2.clicked.connect(button_2_clicked)


        grid = QGridLayout()
        grid.addWidget(self.goal_box)
        grid.addWidget(self.state_box)
        grid.addWidget(self.line_box_1)
        grid.addWidget(self.stop_box)

        self.setLayout(grid)
        self.setWindowTitle("Operator UI")


    def trigger(self):
            print("Trigger ui")
            self.label_goal.setText(Callbacks.goal.to_do)
            self.label_state_doing.setText(Callbacks.state.doing)
            self.label_state_estop.setText(str(Callbacks.state.e_stop))



def main(args=None):

    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            node = OperatorUI()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        t = threading.Thread(target=launch_node_callback_local)
        t.daemon = True
        t.start()

    # Window has to be in the main thread
    def launch_window():
        app = QApplication(sys.argv)
        clock = Window()
        clock.show()
        sys.exit(app.exec_())

    launch_node()
    launch_window()

if __name__ == '__main__':
    main()
