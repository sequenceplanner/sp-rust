import sys
import signal
import rclpy
import time
import csv
import os
import math
import numpy
import ast
import dorna
import json

from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_dorna_msgs.msg import DornaGuiToEsd
from ros2_dorna_msgs.msg import DornaEsdToGui
from ros2_dorna_msgs.msg import Goal
from ros2_dorna_msgs.msg import State

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

class Ros2DornaDriver(Node):

    def __init__(self, homing):
        super().__init__("ros2_dorna_driver")


        config = os.path.join(get_package_share_directory('dorna'), 'config.yaml')
        self.robot = dorna.Dorna(config)

        while True:
            self.get_logger().info('Trying to connect to dorna...')
            ret = self.robot.connect('/dev/ttyDORNA')
            if json.loads(ret)['connection'] == 2:
                self.get_logger().info(ret)
                break
            self.get_logger().info('Connection failed.')
            time.sleep(5) # back off

        self.get_logger().info('Connected!')

        if homing:
            self.get_logger().info('Performing homing procedure...')
            ret = self.robot.home(["j0", "j1", "j2", "j3", "j4"])
            self.get_logger().info(ret)
            self.get_logger().info('Homing done.')


        # check that we are in fact homed
        homed_str = self.robot.homed()
        homed = json.loads(homed_str)
        self.get_logger().info(homed_str)
        if homed['j0'] == 0 or homed['j1'] == 0 or homed['j2'] == 0 or homed['j3'] == 0 or homed['j4'] == 0:
           self.get_logger().error('Dorna not homed! Exiting.')
           sys.exit(1)

        default_speed_str = self.robot.default_speed()
        self.get_logger().info(default_speed_str)
        # the default seems to be 1000
        self.robot.set_default_speed({"joint": 1000})

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_reference_pose = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_tolerance = 0.01

        # get act pos from actual joint values
        self.update_act_pos()

        self.joints_input = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_poses.csv')

        # gui to esd:
        self.gui_to_esd_msg = DornaGuiToEsd()
        self.gui_to_esd_msg.gui_control_enabled = False
        self.gui_to_esd_msg.gui_speed_control = 0
        self.gui_to_esd_msg.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.gui_to_esd_subscriber = self.create_subscription(
            DornaGuiToEsd,
            "/dorna_gui_to_esd",
            self.gui_to_esd_callback,
            10)

        # sp to esd:
        self.sp_to_esd_msg = Goal()
        self.sp_to_esd_msg.ref_pos = ""

        self.sp_to_esd_subscriber = self.create_subscription(
            Goal,
            "/dorna/goal",
            self.sp_to_esd_callback,
            10)

        # esd to gui:
        self.esd_to_gui_msg = DornaEsdToGui()
        self.esd_to_gui_msg.actual_pose = "init"
        self.esd_to_gui_timer_period = 0.1

        self.esd_to_gui_publisher_ = self.create_publisher(
            DornaEsdToGui,
            "/dorna_esd_to_gui",
            10)


        # esd to joints:
        self.joint_names = ["dorna_axis_1_joint",
                            "dorna_axis_2_joint",
                            "dorna_axis_3_joint",
                            "dorna_axis_4_joint",
                            "dorna_axis_5_joint",
                            "nonexistent"]

        self.joint_state_timer_period = 0.05

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/dorna_joint_states",
            10)

        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period,
            self.joint_state_publisher_callback)

        # esd to sp:
        self.esd_to_sp_msg = State()
        self.esd_to_sp_msg.act_pos = ""

        self.esd_to_sp_publisher_ = self.create_publisher(
            State,
            "/dorna/state",
            10)

        # node management
        self.mode = NodeMode()
        self.mode.mode = "init"

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "/dorna/node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "/dorna/mode",
            10)

    def sp_node_cmd_callback(self, data):
        self.node_cmd = data

        # move to general function in sp
        echo_msg = {}
        for k in Goal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(self.sp_to_esd_msg, "_"+k)})

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
        with open(self.joints_input, 'r') as f_in:
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

        with open(self.joints_input, 'r') as joint_csv:
            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
            for row in joint_csv_reader:
                if len(ast.literal_eval(row[1])) == 5 and current_pose != []:
                    saved_pose = ast.literal_eval(row[1])
                    if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tolerance) for i in range(0, 5)):
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
        with open(self.joints_input, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            return pose_list

    def sp_to_esd_callback(self, data):
        if self.gui_to_esd_msg.gui_control_enabled:
            return

        self.sp_to_esd_msg.ref_pos = data.ref_pos
        pose_list = self.read_and_generate_pose_list()
        if self.sp_to_esd_msg.ref_pos in pose_list:
            new_pose = self.get_pose_from_pose_name(self.sp_to_esd_msg.ref_pos)
            if new_pose != self.joint_reference_pose:
                self.get_logger().info("moving to goal pose " + self.sp_to_esd_msg.ref_pos)
                self.move_to_pose_rad(new_pose)
                self.joint_reference_pose = new_pose
        else:
            pass

    def move_to_pose_rad(self, p):
        p_deg = [ p[0] * 180.0/math.pi, p[1] * 180.0/math.pi, p[2] * 180.0/math.pi, p[3] * 180.0/math.pi, p[4] * 180.0/math.pi ]
        command = {"command": "move", "prm":{"path": "joint", "movement":0, "joint": p_deg}}
        #self.get_logger().info(json.dumps(command))

        while True:
            # It seems that the dorna needs to (sometimes) flush
            # something before being issued a new command. Without
            # sleeping a bit here, we frequently get the timeout error
            # below. However, once we get that it seems we cannot
            # recover and need to restart. So for now we are just
            # careful in not overloading the robot instead. Delay may
            # need to be changed, waiting it to break again.
            time.sleep(0.1)
            retval = self.robot.play(command, append = False)
            #self.get_logger().info(retval)

            js = json.loads(retval)
            ok = True
            # json is object on error, list on success...
            if isinstance(js, dict):
                # error case
                if js['error'] == 3:
                    self.get_logger().info('Dorna timeout, retrying...')
                    time.sleep(1)
                    ok = False
                elif js['error'] != None:
                    self.get_logger().error('new kind of error, add handling. exiting')
                    sys.exit(1)
            else:
                # list of commands, assume everything is fine
                pass

            if ok:
                break


    def gui_to_esd_callback(self, data):
        if data.gui_control_enabled and self.gui_to_esd_msg != data:
            self.move_to_pose_rad(data.gui_joint_control)
            self.joint_reference_pose = data.gui_joint_control
        self.gui_to_esd_msg = data

    def update_act_pos(self):
        rp = self.robot.position()
        # self.get_logger().info(rp);
        js = json.loads(rp)
        if len(js) == 5:
            self.act_pos[0] = js[0] * math.pi/180.0
            self.act_pos[1] = js[1] * math.pi/180.0
            self.act_pos[2] = js[2] * math.pi/180.0
            self.act_pos[3] = js[3] * math.pi/180.0
            self.act_pos[4] = js[4] * math.pi/180.0

    def joint_state_publisher_callback(self):
        joint_state = JointState()
        joint_state.name = self.joint_names
        self.update_act_pos()
        joint_state.position = self.act_pos
        # ros wants 6 joints
        joint_state.position.append(0.0)

        self.joint_state_publisher_.publish(joint_state)

        old_pose_name = self.esd_to_gui_msg.actual_pose
        self.esd_to_gui_msg.actual_pose = self.get_pose_name_from_pose()
        if old_pose_name != self.esd_to_gui_msg.actual_pose and "unknown" != self.esd_to_gui_msg.actual_pose:
            self.get_logger().info("reached goal pose " + self.esd_to_gui_msg.actual_pose)
        self.esd_to_gui_publisher_.publish(self.esd_to_gui_msg)

        self.esd_to_sp_msg.act_pos = self.esd_to_gui_msg.actual_pose
        self.esd_to_sp_publisher_.publish(self.esd_to_sp_msg)

    def cleanup(self):
        self.get_logger().info("disconnecting robot")
        self.robot.disconnect()
        self.get_logger().info("terminating robot")
        self.robot.terminate()
        self.get_logger().info("robot terminated")

def main(args=None):
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    homing = False
    if len(command_line_args) > 0 and command_line_args[0] == 'True':
        homing = True
    else:
        homing = False

    rclpy.init(args=args)
    ros2_dorna_driver = Ros2DornaDriver(homing)

    signal.signal(signal.SIGINT, stop_node)
    while rclpy.ok():
        rclpy.spin_once(ros2_dorna_driver)

    ros2_dorna_driver.cleanup()
    ros2_dorna_driver.destroy_node()


def stop_node(*args):
    rclpy.shutdown()
    return True

if __name__ == '__main__':
    try:
        main()
    except:
        stop_node()
