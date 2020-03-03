import sys
import rclpy
import time
import csv
import os
import math
import numpy
import ast
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2_dorna_msgs.msg import DornaGuiToEsd
from ros2_dorna_msgs.msg import DornaEsdToGui
from ros2_dorna_msgs.msg import DornaSPToEsd
from ros2_dorna_msgs.msg import DornaEsdToSP
from ros2_dorna_msgs.msg import DornaSPToEsd
from ros2_dorna_msgs.msg import DornaEsdToSP
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

# NAMESPACE = LaunchConfiguration('node_namespace', default='').perform()

class Ros2DornaSimulator(Node):

    def __init__(self):
        super().__init__("ros2_dorna_simulator")

        self.namespace = ""

        if len(sys.argv) != 2:
            self.namespace = sys.argv[1]
        else:
            pass

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.sync_speed_scale = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.max_speed_factor = 2 # do not exceed 2, for now... Reduce if necessary.
        self.joint_reference_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_tolerance = 0.01

        self.joints_input = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_poses.csv')

        # gui to esd:
        self.gui_to_esd_msg = DornaGuiToEsd()
        self.gui_to_esd_msg.gui_control_enabled = False                       
        self.gui_to_esd_msg.gui_speed_control = 0                             
        self.gui_to_esd_msg.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                      

        self.gui_to_esd_subscriber = self.create_subscription(
            DornaGuiToEsd, 
            "/dorna_gui_to_esd",
            self.gui_to_esd_callback,
            10)
        
        # joints to esd:
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/dorna_joint_states",
            self.joint_state_callback,
            10)

        # sp to esd:
        self.sp_to_esd_msg = DornaSPToEsd()
        self.sp_to_esd_msg.reference_pose = ""
        self.sp_to_esd_msg.reference_joint_speed = 0

        self.sp_to_esd_subscriber = self.create_subscription(
            DornaSPToEsd, 
            "/dorna_sp_to_esd",
            self.sp_to_esd_callback,
            10)

        time.sleep(2)

        # esd to gui:
        self.esd_to_gui_msg = DornaEsdToGui()
        self.esd_to_gui_msg.actual_pose = "init"
        self.esd_to_gui_timer_period = 0.1

        self.esd_to_gui_publisher_ = self.create_publisher(
            DornaEsdToGui,
            "/dorna_esd_to_gui",
            10)

        self.esd_to_gui_timer = self.create_timer(
            self.esd_to_gui_timer_period, 
            self.esd_to_gui_publisher_callback)

        # esd to joints:
        self.joint_state = JointState()
        if self.namespace != "":
            self.joint_names = [self.namespace + "/" + "dorna_axis_1_joint", 
                                self.namespace + "/" + "dorna_axis_2_joint", 
                                self.namespace + "/" + "dorna_axis_3_joint",
                                self.namespace + "/" + "dorna_axis_4_joint", 
                                self.namespace + "/" + "dorna_axis_5_joint",
                                "nonexistent"]
        else:
            self.joint_names = ["dorna_axis_1_joint", 
                                "dorna_axis_2_joint", 
                                "dorna_axis_3_joint",
                                "dorna_axis_4_joint", 
                                "dorna_axis_5_joint",
                                "nonexistent"]

        self.joint_state_timer_period = 0.01

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            "/dorna_joint_states",
            10)
    
        self.joint_state_publisher_timer = self.create_timer(
            self.joint_state_timer_period, 
            self.joint_state_publisher_callback)

        # esd to sp:
        self.esd_to_sp_msg = DornaEsdToSP()
        self.esd_to_sp_msg.actual_pose = ""
        self.esd_to_sp_msg.actual_joint_speed = 0
        self.esd_to_sp_msg.echo_reference_pose = ""
        self.esd_to_sp_msg.echo_reference_joint_speed = 0
        self.esd_to_sp_timer_period = 0.1

        self.esd_to_sp_publisher_ = self.create_publisher(
            DornaEsdToSP,
            "/dorna_esd_to_sp",
            10)

        self.esd_to_sp_publisher_timer = self.create_timer(
            self.esd_to_sp_timer_period, 
            self.esd_to_sp_callback)

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
                if len(ast.literal_eval(row[1])) == 6 and current_pose != []:
                    saved_pose = ast.literal_eval(row[1])
                    if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tolerance) for i in range(0, 6)):
                        actual_joint_pose = row[0]
                        break
                    else:
                        actual_joint_pose = "UNKNOWN"
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

    def joint_state_callback(self, data):
        self.act_pos[0] = data.position[0]
        self.act_pos[1] = data.position[1]
        self.act_pos[2] = data.position[2]
        self.act_pos[3] = data.position[3]
        self.act_pos[4] = data.position[4]
        # self.act_pos[5] = data.position[5]

    def sp_to_esd_callback(self, data):
        self.sp_to_esd_msg.reference_pose = data.reference_pose
        self.sp_to_esd_msg.reference_joint_speed = data.reference_joint_speed
        if self.sp_to_esd_msg.reference_pose in self.read_and_generate_pose_list():
            self.joint_reference_pose = self.get_pose_from_pose_name(self.sp_to_esd_msg.reference_pose)
        else:
            pass

    def gui_to_esd_callback(self, data):
        self.gui_to_esd_msg.gui_control_enabled = data.gui_control_enabled
        self.gui_to_esd_msg.gui_speed_control = data.gui_speed_control
        self.gui_to_esd_msg.gui_joint_control[0] = round(data.gui_joint_control[0], 3)
        self.gui_to_esd_msg.gui_joint_control[1] = round(data.gui_joint_control[1], 3)
        self.gui_to_esd_msg.gui_joint_control[2] = round(data.gui_joint_control[2], 3)
        self.gui_to_esd_msg.gui_joint_control[3] = round(data.gui_joint_control[3], 3)
        self.gui_to_esd_msg.gui_joint_control[4] = round(data.gui_joint_control[4], 3)
        # self.gui_to_esd_msg.gui_joint_control[5] = round(data.gui_joint_control[5], 3)

    def joint_state_publisher_callback(self):        
        if self.gui_to_esd_msg.gui_control_enabled == True:
            if self.gui_to_esd_msg.gui_joint_control != None:
            
                for i in range(0, 6):
                    self.sync_speed_scale[i] = abs(self.gui_to_esd_msg.gui_joint_control[i] - self.act_pos[i])

                self.sync_max = max(self.sync_speed_scale)
                if self.sync_max != 0:
                    self.sync_max_factor = 1 / self.sync_max
                else:
                    self.sync_max_factor = 1

                for i in range(0, 6):
                    self.sync_speed_scale[i] = self.sync_speed_scale[i]*self.sync_max_factor

                for i in range(0, 6):
                    # print(self.sync_speed_scale)
                    if self.gui_to_esd_msg.gui_joint_control[i] < self.act_pos[i] - 0.001*self.max_speed_factor:
                        if self.gui_to_esd_msg.gui_joint_control[i] < self.act_pos[i] - 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.0001*self.max_speed_factor*self.gui_to_esd_msg.gui_speed_control*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] - 0.001*self.max_speed_factor
                    elif self.gui_to_esd_msg.gui_joint_control[i] > self.act_pos[i] + 0.001*self.max_speed_factor:
                        if self.gui_to_esd_msg.gui_joint_control[i] > self.act_pos[i] + 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.0001*self.max_speed_factor*self.gui_to_esd_msg.gui_speed_control*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] + 0.001*self.max_speed_factor
                    else:
                        self.pub_pos[i] = self.gui_to_esd_msg.gui_joint_control[i]
                        pass
            else:
                pass
        else:

            if self.joint_reference_pose != None:
                for i in range(0, 6):
                    self.sync_speed_scale[i] = abs(self.joint_reference_pose[i] - self.act_pos[i])

                self.sync_max = max(self.sync_speed_scale)
                if self.sync_max != 0:
                    self.sync_max_factor = 1 / self.sync_max
                else:
                    self.sync_max_factor = 1

                for i in range(0, 6):
                    self.sync_speed_scale[i] = self.sync_speed_scale[i]*self.sync_max_factor

            
                for i in range(0, 6):
                    if self.joint_reference_pose[i] < self.act_pos[i] - 0.001*self.max_speed_factor:
                        if self.joint_reference_pose[i] < self.act_pos[i] - 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] - 0.0001*self.max_speed_factor*self.sp_to_esd_msg.reference_joint_speed*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] - 0.001*self.max_speed_factor
                    elif self.joint_reference_pose[i] > self.act_pos[i] + 0.001*self.max_speed_factor:
                        if self.joint_reference_pose[i] > self.act_pos[i] + 0.01:
                            self.pub_pos[i] = round(self.act_pos[i] + 0.0001*self.max_speed_factor*self.sp_to_esd_msg.reference_joint_speed*self.sync_speed_scale[i], 4)
                        else:
                            self.pub_pos[i] = self.act_pos[i] + 0.001*self.max_speed_factor
                    else:
                        self.pub_pos[i] = self.joint_reference_pose[i]
            else:
                pass

        self.joint_state.name = self.joint_names
        self.joint_state.position = self.pub_pos
        self.joint_state_publisher_.publish(self.joint_state)
            
    def esd_to_gui_publisher_callback(self):
        self.esd_to_gui_msg.actual_pose = self.get_pose_name_from_pose()
        self.esd_to_gui_publisher_.publish(self.esd_to_gui_msg)

    def esd_to_sp_callback(self):
        self.esd_to_sp_msg.actual_pose = self.esd_to_gui_msg.actual_pose
        self.esd_to_sp_msg.actual_joint_speed = self.sp_to_esd_msg.reference_joint_speed
        self.esd_to_sp_msg.echo_reference_pose = self.sp_to_esd_msg.reference_pose
        self.esd_to_sp_msg.echo_reference_joint_speed = self.sp_to_esd_msg.reference_joint_speed
        self.esd_to_sp_publisher_.publish(self.esd_to_sp_msg)
    
def main(args=None):

    rclpy.init(args=args)
    ros2_dorna_simulator = Ros2DornaSimulator()
    rclpy.spin(ros2_dorna_simulator)
    ros2_dorna_simulator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()