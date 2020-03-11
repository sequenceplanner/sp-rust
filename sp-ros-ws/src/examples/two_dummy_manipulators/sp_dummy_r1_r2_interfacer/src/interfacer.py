from sensor_msgs.msg import JointState
from dummy_robot_messages.msg import State
from dummy_robot_messages.msg import Control
import rclpy
from rclpy.node import Node
import time
import random

class Interfacer(Node):

    # we reset after 10 seconds of idle
    def reset(self):
        self.active = False
        self.act_pos = "away"
        self.ref_pos = "away"
        self.act_pos_j1 = 0.0
        self.ref_pos_j1 = 0.0
        self.act_pos_j2 = 0.0
        self.ref_pos_j2 = 0.0
        self.to_r.position = [0.0, 0.0]
        self.echo.ref_pos = "away"
        self.echo.activate = False

    def __init__(self, r_name):
        super().__init__("{}_interfacer".format(r_name))

        self.logger = self.get_logger()

        self.r_name = r_name

        self.from_r = JointState()
        self.to_r = JointState()
        self.tmr_period = 0.02
        self.tmr_period2 = 0.5
        self.counter_period = 1
        self.idle_counter = 0
        self.echo = Control()

        self.reset()

        self.to_r.name = ["{}_j1".format(self.r_name), "{}_j2".format(self.r_name)]
        self.to_r.position = [0.0, 0.0]

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.joint_subscriber = self.create_subscription(
            JointState,
            "/{}_joint_states".format(self.r_name),
            self.joint_callback,
            10)

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.cmd_subscriber = self.create_subscription(
            Control,
            "/drm/{}/Control".format(self.r_name),
            self.sp_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        self.state_to_sp_publisher_ = self.create_publisher(
            State,
            "/drm/{}/State".format(self.r_name),
            10)

        self.joint_cmd_publisher_ = self.create_publisher(
            JointState,
            "/{}_joint_states".format(self.r_name),
            10)

        # Decouple message receiving and forwarding
        self.interfacer_to_sp_tmr = self.create_timer(
            self.tmr_period2,
            self.interfacer_to_sp_publisher_callback)

        self.interfacer_to_r_tmr = self.create_timer(
            self.tmr_period,
            self.interfacer_to_r_publisher_callback)

        self.idle_timer = self.create_timer(
            1.0,
            self.idle_timer_callback)

    def idle_timer_callback(self):
        self.idle_counter += 1
        if self.idle_counter > 10:
            self.idle_counter = 0
            self.reset()

    def joint_callback(self, data):
        self.act_pos_j1 = data.position[0]
        self.act_pos_j2 = data.position[1]
        if self.act_pos_j1 == 0.0 and self.act_pos_j2 == 0.0:
            self.act_pos = "away"
        elif self.act_pos_j1 == -1.0 and self.act_pos_j2 == -1.0:
            self.act_pos = "at"
        else:
            self.act_pos = "unknown"

    def sp_callback(self, data):
        self.idle_counter = 0
        self.active = data.activate
        self.ref_pos = data.ref_pos
        self.echo = data

        if self.ref_pos == "at":
            self.ref_pos_j1 = -1.0
            self.ref_pos_j2 = -1.0
        elif self.ref_pos == "away":
            self.ref_pos_j1 = 0.0
            self.ref_pos_j2 = 0.0
        else:
            pass

    def interfacer_to_r_publisher_callback(self):
        if self.active and self.ref_pos_j1 < self.act_pos_j1:
            if self.ref_pos_j2 < self.act_pos_j2:
                self.to_r.position = [round(self.act_pos_j1 - 0.01, 2), round(self.act_pos_j2 - 0.01, 2)]
            elif self.ref_pos_j2 > self.act_pos_j2:
                self.to_r.position = [round(self.act_pos_j1 - 0.01, 2), round(self.act_pos_j2 + 0.01, 2)]
            else:
                pass
        elif self.active and self.ref_pos_j1 > self.act_pos_j1:
            if self.ref_pos_j2 < self.act_pos_j2:
                self.to_r.position = [round(self.act_pos_j1 + 0.01, 2), round(self.act_pos_j2 - 0.01, 2)]
            elif self.ref_pos_j2 > self.act_pos_j2:
                self.to_r.position = [round(self.act_pos_j1 + 0.01, 2), round(self.act_pos_j2 + 0.01, 2)]
            else:
                pass
        else:
            pass

        self.joint_cmd_publisher_.publish(self.to_r)



    def interfacer_to_sp_publisher_callback(self):
        msg = State()
        msg.act_pos = self.act_pos
        msg.active = self.active
        self.state_to_sp_publisher_.publish(msg)
