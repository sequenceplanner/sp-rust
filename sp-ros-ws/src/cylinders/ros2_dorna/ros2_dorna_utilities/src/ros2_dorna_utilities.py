import os
import sys
import rclpy
import time
import csv
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ros2_dorna_msgs.msg import DornaUtilsToGui
from ros2_dorna_msgs.msg import DornaGuiToUtils
from ros2_dorna_msgs.msg import DornaSPToUtils
from ros2_dorna_msgs.msg import DornaUtilsToSP
from ament_index_python.packages import get_package_share_directory

class Ros2DornaUtilities(Node):

    def __init__(self):
        super().__init__("ros2_dorna_utilities")

        self.namespace = ""

        if len(sys.argv) != 2:
            self.namespace = sys.argv[1]
        else:
            pass

        self.utils_to_gui_msg = DornaUtilsToGui()
        self.joint_state = JointState()

        self.utils_to_gui_timer_period = 0.5

        # self.utils_to_esd_msg.actual_pose = ""
        self.utils_to_gui_msg.saved_poses = []
        # self.utils_to__msg.echo_utility_action = ""
        # self.utils_to_esd_msg.echo_utility_pose_name = ""

        self.act_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.prev_action = ""
        self.prev_pose_name = ""

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

        self.joints_input = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_poses.csv')
        self.joints_oldposes = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_oldposes.csv')
        self.joints_newposes = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_newposes.csv')

        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/dorna_joint_states",
            self.joint_callback,
            10)

        self.gui_to_utils_subscriber = self.create_subscription(
            DornaGuiToUtils, 
            "/dorna_gui_to_utils",
            self.gui_to_utils_callback,
            10)

        time.sleep(2)

        self.utils_to_gui_publisher_ = self.create_publisher(
            DornaUtilsToGui,
            "/dorna_utils_to_gui",
            10)

        self.utils_to_gui_timer = self.create_timer(
            self.utils_to_gui_timer_period, 
            self.utils_to_gui_callback)

    def delete_pose(self, pose_name):
        '''
        Delete one pose from pose lists.
        '''
        
        with open(self.joints_input, "r") as f_in, open(self.joints_newposes, "w") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] != pose_name:
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass
        
        os.remove(self.joints_input)
        os.rename(self.joints_newposes, self.joints_input)
        self.last_completed_action = 'deleted: ' + str(pose_name)


    def clear_pose_list(self):
        '''
        Delete all poses from pose lists.
        '''
        
        with open(self.joints_input, "r") as f_in, open(self.joints_newposes, "w") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == 'control_pose':
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass

        os.remove(input_f)
        os.rename(newpose_f, input_f)
        self.last_completed_action = 'pose lists cleared'


    def update_pose_list(self, input_f, oldpose_f, newpose_f, name, function):
        '''
        Update a pose list by appending a new pose or updating an existing one.
        '''
        
        with open(input_f, 'r') as csv_read:
            csv_reader = csv.reader(csv_read, delimiter=':')
            if all((row[0] != name) for row in csv_reader):
                self.append_new_pose(input_f, name, function)
                self.last_completed_action = 'appended: ' + str(name)
            else:
                self.update_split(input_f, oldpose_f, newpose_f, name, function)
                self.last_completed_action = 'updated: ' + str(name)

        
    def read_and_generate_pose_list(self, input_f):
        '''
        Acquire the names of all saved poses from a file to a list.
        '''

        pose_list = []
        with open(input_f, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            return pose_list


    def update_split(self, input_f, oldpose_f, newpose_f, name, pose):
        '''
        Update the values of an existing pose in a pose list by saving it in a newpose file. The poses that are not 
        being updated are saved in a oldpose list and those two lists are merged in the update_merge method. 
        '''

        with open(input_f, "r") as f_in, open(oldpose_f, "w") as f_op, open(newpose_f, "w") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_merge(input_f, newpose_f, oldpose_f)


    def update_merge(self, input_f, newpose_f, oldpose_f):
        '''
        Merge an olpose and a newpose list and thus form an updated pose list. 
        '''

        with open(newpose_f, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(oldpose_f, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(input_f)
        os.remove(newpose_f)
        os.rename(oldpose_f, input_f)


    def append_new_pose(self, file, name, pose):
        '''
        Append a nonexisting pose to a pose list.
        '''

        with open(file, 'a') as csv_append:
            csv_appender = csv.writer(csv_append, delimiter=':')
            csv_appender.writerow([name, pose])

    
    def inhibit_tick(self):
        '''
        Check if two successive messages are the same and disallow consumption if True.
        This method assigns values to the previous message variables so that they can be compared later.
        '''

        self.prev_action = self.action
        self.prev_pose_name = self.pose_name
    

    def uninhibit_tick(self):
        '''
        Continue reading the ticked data
        '''

        self.prev_action = "reset"
        self.prev_pose_name = "reset"


    def joint_callback(self, data):
        '''
        Continuously read joint data
        '''

        self.act_pos[0] = data.position[0]
        self.act_pos[1] = data.position[1]
        self.act_pos[2] = data.position[2]
        self.act_pos[3] = data.position[3]
        self.act_pos[4] = data.position[4]
        # self.act_pos[5] = data.position[5]

    def gui_to_utils_callback(self, data):
        '''
        Evaluate and consume command messages from Sequence Planner
        '''

        self.action = data.utility_action
        self.pose_name = data.utility_pose_name

        if self.action == self.prev_action and \
           self.pose_name == self.prev_pose_name:
            self.tick_inhibited = True
        else:
            self.tick_inhibited = False

        if self.pose_name == "RESET":
            self.uninhibit_tick()

        if self.tick_inhibited == False:
            self.inhibit_tick()
            if self.action == "delete":
                self.delete_pose(self.pose_name)
            elif self.action == "clear":
                self.clear_pose_list()
            elif self.action == "update":
                self.update_pose_list(self.joints_input, self.joints_oldposes, self.joints_newposes, self.pose_name, self.act_pos)
            else:
                self.uninhibit_tick()
        else:
            pass

    def utils_to_gui_callback(self):
        self.utils_to_gui_msg.saved_poses = self.read_and_generate_pose_list(self.joints_input)
        self.utils_to_gui_publisher_.publish(self.utils_to_gui_msg)

def main(args=None):
    rclpy.init(args=args)

    ros2_dorna_utilities = Ros2DornaUtilities()

    rclpy.spin(ros2_dorna_utilities)

    ros2_dorna_utilities.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()