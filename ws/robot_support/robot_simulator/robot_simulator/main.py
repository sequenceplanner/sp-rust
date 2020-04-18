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

from robot_msgs.msg import GuiToRobot
from robot_msgs.msg import RobotToGui
from robot_msgs.msg import RobotGoal
from robot_msgs.msg import RobotState

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode


class RobotSimulator(Node):

    def __init__(self):
        super().__init__(node_name= "robot_simulator", automatically_declare_parameters_from_overrides=True)

        self.saved_poses_file = self.get_parameter("saved_poses_file")
        self.joint_names = self.get_parameter("joint_names")
        self.no_of_joints = len(self.joint_names.value)
        self.act_pos = self.get_parameter_or("act_pos", [0.0]*self.no_of_joints)
        self.pub_pos = self.get_parameter_or("pub_pos", [0.0]*self.no_of_joints)
        self.sync_speed_scale = self.get_parameter_or("sync_speed_scale", [1.0]*self.no_of_joints)
        self.max_speed_factor = self.get_parameter_or("max_speed_factor", 4.0)
        self.joint_reference_pose = self.get_parameter_or("joint_reference_pose", [0.0]*self.no_of_joints)
        self.joint_tolerance = self.get_parameter_or("joint_tolerance", 0.01)
        self.joint_state_timer_period = self.get_parameter_or("joint_state_timer_period", 1.0)
        self.robot_to_gui_timer_period = self.get_parameter_or("robot_to_gui_timer_period", 0.1)
        self.robot_state_timer_period = self.get_parameter_or("robot_state_timer_period", 0.1)



        # gui to esd:
        self.gui_to_robot = GuiToRobot()
        self.gui_to_robot.gui_control_enabled = False
        self.gui_to_robot.gui_speed_control = 0
        self.gui_to_robot.gui_joint_control = [0.0]*self.no_of_joints

        self.gui_to_robot_subscriber = self.create_subscription(
            GuiToRobot,
            "gui_to_robot",
            self.gui_to_robot_callback,
            10)

        # joints to esd:
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "poses_cmd",
            self.joint_state_callback,
            10)

        # sp to esd:
        self.robot_goal_msg = RobotGoal()
        self.robot_goal_msg.ref_pos = self.get_pose_name_from_pose()

        self.robot_goal_subscriber = self.create_subscription(
            RobotGoal,
            "goal",
            self.robot_goal_callback,
            10)


        # esd to gui:
        self.robot_to_gui = RobotToGui()
        self.robot_to_gui.actual_pose = "init"

        self.robot_to_gui_publisher = self.create_publisher(
            RobotToGui,
            "robot_to_gui",
            10)

        self.esd_to_gui_timer = self.create_timer(
            self.robot_to_gui_timer_period,
            self.callback)

        # esd to joints:
        self.joint_state = JointState()

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "joint_states",
            10)

        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period,
            self.joint_state_publisher_callback)

        # esd to sp:
        self.robot_state = RobotState()
        self.robot_state.act_pos = ""

        self.robot_state_publisher_ = self.create_publisher(
            RobotState,
            "state",
            10)

        self.robot_state_publisher_timer = self.create_timer(
            self.robot_state_timer_period,
            self.robot_state_callback)


        # node management
        self.mode = NodeMode()
        self.mode.mode = "init"

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

        # refresh poses in case gui was used to move the robot
        self.robot_goal_msg.ref_pos = self.get_pose_name_from_pose()
        self.robot_to_gui.actual_pose = self.get_pose_name_from_pose()

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


    def get_pose_from_pose_name(self, name):
        '''
        Returns the saved pose that matches the pose name
        '''

        pose = []
        if self.saved_poses_file.value:
            with open(self.saved_poses_file.value, 'r') as f_in:
                csv_reader = csv.reader(f_in, delimiter=':')
                for row in csv_reader:
                    if name == row[0]:
                        pose = ast.literal_eval(row[1])
                        break
                    else:
                        pass

        if pose != []:
            self.pose_name_error = ''
            return pose
        else:
            self.pose_name_error = 'pose with the name ' + name + ' not saved'
            return []

    def get_pose_name_from_pose(self):
        '''
        While all joint velocities are 0, compare current robot joint pose with saved poses in the joint_csv file
        and return the name of the saved pose if they match with a tolerance, else return unknown as the
        current joint pose. Using joint_callback because it is very slow to get_current_joint_values via
        moveit_commander according to KCacheGrind.
        '''

        actual_joint_pose = ""
        current_pose = self.act_pos

        if self.saved_poses_file.value:
            with open(self.saved_poses_file.value, 'r') as joint_csv:
                joint_csv_reader = csv.reader(joint_csv, delimiter=':')
                for row in joint_csv_reader:
                    if len(ast.literal_eval(row[1])) == self.no_of_joints and current_pose != []:
                        saved_pose = ast.literal_eval(row[1])
                        if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tolerance) for i in range(0, self.no_of_joints)):
                            actual_joint_pose = row[0]
                            break
                        else:
                            actual_joint_pose = "unknown"
                            pass
                    else:
                        pass

        return actual_joint_pose

    def read_and_generate_pose_list(self):
        '''
        Acquire the names of all saved poses from a file to a list.
        '''

        pose_list = []
        with open(self.saved_poses_file, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            return pose_list

    def joint_state_callback(self, data):
        for i in range(self.no_of_joints):
            self.act_pos[i] = data.position[i]

    def robot_goal_callback(self, data):
        self.robot_goal_msg.ref_pos = data.ref_pos
        if self.robot_goal_msg.ref_pos in self.read_and_generate_pose_list():
            self.joint_reference_pose = self.get_pose_from_pose_name(self.robot_goal_msg.ref_pos)
        else:
            pass

    def gui_to_robot_callback(self, data):
        self.gui_to_robot.gui_control_enabled = data.gui_control_enabled
        self.gui_to_robot.gui_speed_control = data.gui_speed_control
        for i in range(self.no_of_joints):
            self.gui_to_robot.gui_joint_control[i] = round(data.gui_joint_control[i], 3)
        if self.gui_to_robot.gui_control_enabled == True:
            # when gui control is enabled, copy gui ref to sp ref so we don't
            # move away after releasing gui control again.
            self.joint_reference_pose = self.gui_to_robot.gui_joint_control

    def joint_state_publisher_callback(self):
        if self.gui_to_robot.gui_control_enabled == True:
            if self.gui_to_robot.gui_joint_control != None:

                for i in range(self.no_of_joints):
                    self.sync_speed_scale[i] = abs(self.gui_to_robot.gui_joint_control[i] - self.act_pos[i])

                self.sync_max = max(self.sync_speed_scale)
                if self.sync_max != 0:
                    self.sync_max_factor = 1 / self.sync_max
                else:
                    self.sync_max_factor = 1

                for i in range(self.no_of_joints):
                    self.sync_speed_scale[i] = self.sync_speed_scale[i]*self.sync_max_factor

                for i in range(self.no_of_joints):
                    if self.gui_to_robot.gui_joint_control[i] < self.act_pos[i] - 0.001*self.max_speed_factor:
                        if self.gui_to_robot.gui_joint_control[i] < self.act_pos[i] - 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.0001*self.max_speed_factor*self.gui_to_robot.gui_speed_control*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] - 0.001*self.max_speed_factor
                    elif self.gui_to_robot.gui_joint_control[i] > self.act_pos[i] + 0.001*self.max_speed_factor:
                        if self.gui_to_robot.gui_joint_control[i] > self.act_pos[i] + 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.0001*self.max_speed_factor*self.gui_to_robot.gui_speed_control*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] + 0.001*self.max_speed_factor
                    else:
                        self.pub_pos[i] = self.gui_to_robot.gui_joint_control[i]
                        pass
            else:
                pass
        else:

            if self.joint_reference_pose != None:
                for i in range(self.no_of_joints):
                    self.sync_speed_scale[i] = abs(self.joint_reference_pose[i] - self.act_pos[i])

                self.sync_max = max(self.sync_speed_scale)
                if self.sync_max != 0:
                    self.sync_max_factor = 1 / self.sync_max
                else:
                    self.sync_max_factor = 1

                for i in range(self.no_of_joints):
                    self.sync_speed_scale[i] = self.sync_speed_scale[i]*self.sync_max_factor


                for i in range(self.no_of_joints):
                    if self.joint_reference_pose[i] < self.act_pos[i] - 0.001*self.max_speed_factor:
                        if self.joint_reference_pose[i] < self.act_pos[i] - 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.01*self.max_speed_factor*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] - 0.001*self.max_speed_factor
                    elif self.joint_reference_pose[i] > self.act_pos[i] + 0.001*self.max_speed_factor:
                        if self.joint_reference_pose[i] > self.act_pos[i] + 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.01*self.max_speed_factor*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] + 0.001*self.max_speed_factor
                    else:
                        self.pub_pos[i] = self.joint_reference_pose[i]
            else:
                pass

        self.joint_state.name = self.joint_names.value
        self.joint_state.position = self.pub_pos
        self.joint_state_publisher_.publish(self.joint_state)

    def callback(self):
        old_pose_name = self.robot_to_gui.actual_pose
        self.robot_to_gui.actual_pose = self.get_pose_name_from_pose()
        if old_pose_name != self.robot_to_gui.actual_pose and "unknown" != self.robot_to_gui.actual_pose:
            self.get_logger().info("reached goal pose " + self.robot_to_gui.actual_pose)
        self.robot_to_gui_publisher.publish(self.robot_to_gui)

    def robot_state_callback(self):
        self.robot_state.act_pos = self.robot_to_gui.actual_pose
        self.robot_state_publisher_.publish(self.robot_state)

def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()

    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
