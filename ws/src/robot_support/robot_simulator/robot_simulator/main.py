import sys
import rclpy
import time
import csv
import os
import math
import numpy
import ast
import json

from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_msgs.msg import GuiToRobot, RobotGoal, RobotState

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode


class RobotSimulator(Node):

    def __init__(self):
        super().__init__(
            node_name="robot_simulator"
        )

        self.saved_poses_file = self.declare_parameter("saved_poses_file").value
        self.joint_names = self.declare_parameter("joint_names").value
        self.no_of_joints = len(self.joint_names)

        self.max_joint_speed = self.declare_parameter("max_joint_speed", value=[45]*self.no_of_joints).value  #deg/sec
        self.speed_scale = self.declare_parameter("speed_scale", value=100).value
        self.joint_tolerance = self.declare_parameter("joint_tolerance", value=0.01).value
        
        self.ref_pos = self.declare_parameter("ref_pos", value=[0.0]*self.no_of_joints).value
        self.act_pos = self.declare_parameter("act_pos", value=[0.0]*self.no_of_joints).value

        self.joint_state_timer_period = self.declare_parameter("joint_state_timer_period", value=0.1).value
        self.robot_state_timer_period = self.declare_parameter("robot_state_timer_period", value=0.5).value

        self.poses = self.load_poses(self.saved_poses_file)


        # gui to robot:
        self.gui_to_robot = GuiToRobot()
        self.gui_to_robot.gui_control_enabled = False
        self.gui_to_robot.gui_speed_control = 0
        self.gui_to_robot.gui_joint_control = self.ref_pos

        self.gui_to_robot_subscriber = self.create_subscription(
            GuiToRobot,
            "gui_to_robot",
            self.gui_to_robot_callback,
            10)

        # esd to joints:
        self.joint_state = JointState()

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "joint_states",
            10)

        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period,
            self.joint_state_ticker)


        # sp to robot:
        self.robot_goal_msg = RobotGoal()
        self.robot_goal_msg.ref_pos = self.pose_from_position(self.poses, self.ref_pos)

        self.robot_goal_subscriber = self.create_subscription(
            RobotGoal,
            "goal",
            self.robot_goal_callback,
            10)

        # robot to sp:
        self.robot_state_msg = RobotState()
        self.robot_state_msg.act_pos = self.pose_from_position(self.poses, self.act_pos)

        self.robot_state_publisher_ = self.create_publisher(
            RobotState,
            "state",
            10)

        self.robot_state_publisher_timer = self.create_timer(
            self.robot_state_timer_period,
            self.robot_state_ticker)


  
        # node management
        self.mode = NodeMode()
        self.mode.mode = "init"
        self.node_cmd = NodeCmd()

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "mode",
            10)

    def sp_node_cmd_callback(self, data):
        '''
        Handles the handshaking with sp
        '''
        self.node_cmd = data
        self.robot_goal_msg.ref_pos = self.pose_from_position(self.poses, self.ref_pos)

        # move to general function in sp
        echo_msg = {}
        for k in RobotGoal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(self.robot_goal_msg, "_"+k)})

        self.mode.echo = json.dumps(echo_msg)

        if self.node_cmd.mode == "run":
            self.mode.mode = "running"
        else:
            self.mode.mode = "init"

        self.sp_mode_publisher.publish(self.mode)


    def load_poses(self, file):
        result = {}
        with open(file, 'r') as joint_csv:
            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
            for row in joint_csv_reader:
                if len(row) == 2 and len(ast.literal_eval(row[1])) == self.no_of_joints:
                    result[row[0]] = ast.literal_eval(row[1])
            
        return result

    def pose_from_position(self, poses, joints):
        result = "unknown"
        for pose, position in poses.items():
            if len(position) != len(joints):
                continue
            if all(numpy.isclose(position[i], joints[i], atol=self.joint_tolerance) for i in range(len(joints))):
                result = pose
                break
        return result

    # KB: Not sure why this was needed. Maybe add it back if we do
    # def joint_state_callback(self, data):
    #     for i in range(self.no_of_joints):
    #         self.act_pos[i] = data.position[i]

    def robot_goal_callback(self, data):
        self.robot_goal_msg = data
        if not self.gui_to_robot.gui_control_enabled and self.robot_goal_msg.ref_pos in self.poses:
            self.ref_pos = self.poses[self.robot_goal_msg.ref_pos]


    def gui_to_robot_callback(self, data):
        self.poses = self.load_poses(self.saved_poses_file)
        self.gui_to_robot = data
        # for i in range(self.no_of_joints):
        #     j = 0.0
        #     if len(data.gui_joint_control) > i:
        #         j = data.gui_joint_control[i]
        #     self.gui_to_robot.gui_joint_control[i] = round(j, 5)

        self.speed_scale = self.gui_to_robot.gui_speed_control        

        if self.gui_to_robot.gui_control_enabled:
            self.ref_pos = self.gui_to_robot.gui_joint_control
        elif self.robot_goal_msg.ref_pos in self.poses:
            self.ref_pos = list(map(lambda x: x*math.pi/180, self.poses[self.robot_goal_msg.ref_pos]))

    def joint_state_ticker(self):

        # Scale the speed of the joints for nice movement
        sync_speed_scale = list(map(lambda i: abs(self.ref_pos[i] - self.act_pos[i]), range(self.no_of_joints)))
        sync_max = max(sync_speed_scale)
        sync_max_factor = 1
        if sync_max != 0:
            sync_max_factor = 1 / sync_max
        sync_speed_scale = list(map(lambda i: sync_speed_scale[i]*sync_max_factor, range(self.no_of_joints)))

        for i in range(self.no_of_joints):
            rad_sec = self.max_joint_speed[i] * math.pi / 180
            step = rad_sec * self.joint_state_timer_period

            if (self.ref_pos[i] < (self.act_pos[i] - step)) and sync_speed_scale[i] > 0.1:
                self.act_pos[i] = round(self.act_pos[i] - (step * sync_speed_scale[i]), 5)
            elif (self.ref_pos[i] > (self.act_pos[i] + step)) and sync_speed_scale[i] > 0.1:
                self.act_pos[i] = round(self.act_pos[i] + (step * sync_speed_scale[i]), 5)
            else:
                self.act_pos[i] = self.ref_pos[i]

        self.joint_state.name = self.joint_names
        self.joint_state.position = self.act_pos
        self.joint_state_publisher_.publish(self.joint_state)

    def robot_state_ticker(self):
        self.robot_state_msg.act_pos = self.pose_from_position(self.poses, self.act_pos)
        self.robot_state_publisher_.publish(self.robot_state_msg)



def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()

    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
