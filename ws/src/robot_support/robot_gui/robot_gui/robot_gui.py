import sys
import time
import ast
import csv
import os
import math
import threading
import numpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from robot_msgs.msg import GuiToRobot, RobotToGui
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class CommVariables():
    trigger_node = None
    trigger_ui = None

    to_robot = GuiToRobot()
    from_robot = JointState()

    to_robot.gui_control_enabled = False
    to_robot.gui_speed_control = 50
    to_robot.gui_joint_control = []

    node_name = ""
    
    saved_poses_file = []
    joint_names = []
    joint_limits = {}
    no_of_joints = 0
    joint_tolerance = 0.01
    poses = {}

def load_poses(file):
    result = {}
    with open(file, 'r') as joint_csv:
        joint_csv_reader = csv.reader(joint_csv, delimiter=':')
        for row in joint_csv_reader:
            if len(row) == 2 and len(ast.literal_eval(row[1])) == CommVariables.no_of_joints:
                result[row[0]] = ast.literal_eval(row[1])
        
    return result

def pose_from_position(poses, joints):
    result = "unknown"
    for pose, position in poses.items():
        if len(position) != len(joints):
            continue
        if all(numpy.isclose(position[i], joints[i], atol=CommVariables.joint_tolerance) for i in range(len(joints))):
            result = pose
            break
    return result

    

class RobotGUI(Node, CommVariables):

    def __init__(self):
        super().__init__(
            node_name= "robot_gui", 
            automatically_declare_parameters_from_overrides=True
        )

        CommVariables.trigger_node = self.trigger

        CommVariables.node_name = self.get_name()
        CommVariables.saved_poses_file = self.get_parameter("saved_poses_file")
        CommVariables.joint_names = self.get_parameter("joint_names")
        CommVariables.joint_limits = self.get_parameter("joint_limits")
        CommVariables.joint_tolerance = self.get_parameter_or("joint_tolerance", 0.01)

        CommVariables.no_of_joints = len(self.joint_names.value)
        CommVariables.poses = load_poses(CommVariables.saved_poses_file.value)



        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10)

        self.gui_to_robot_publisher = self.create_publisher(
            GuiToRobot,
            "gui_to_robot",
            10)

        print("print HEJ gui node")

        time.sleep(2)

        CommVariables.trigger_ui()


    def trigger(self):
        print("trigger node")
        self.gui_to_robot_publisher.publish(CommVariables.to_robot)

    # def tri(self):
    #     self.gui_to_esd_msg.gui_control_enabled = CommVariables.gui_control_enabled

    #     if CommVariables.speed_slider_value >= 0 and CommVariables.speed_slider_value <= 100:
    #         self.gui_to_esd_msg.gui_speed_control = CommVariables.speed_slider_value
    #     else:
    #         pass

    #     self.gui_to_esd_msg.gui_joint_control[0] = CommVariables.slider_1_value * math.pi / 180
    #     self.gui_to_esd_msg.gui_joint_control[1] = CommVariables.slider_2_value * math.pi / 180
    #     self.gui_to_esd_msg.gui_joint_control[2] = CommVariables.slider_3_value * math.pi / 180
    #     self.gui_to_esd_msg.gui_joint_control[3] = CommVariables.slider_4_value * math.pi / 180
    #     self.gui_to_esd_msg.gui_joint_control[4] = CommVariables.slider_5_value * math.pi / 180

    #     self.gui_to_esd_publisher_.publish(self.gui_to_esd_msg)


    def joint_state_callback(self, data):
        CommVariables.from_robot = data
        CommVariables.trigger_ui()





class Window(QWidget, CommVariables):
    triggerSignal = pyqtSignal()

    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        CommVariables.trigger_ui = self.trigger
        self.triggerSignal.connect(self.update_state_variables)

        print("HEJ from window")

        self.loaded = False

    def trigger(self):
        self.triggerSignal.emit()

    def update_state_variables(self):
        if not self.loaded:
            self.loaded = True
            self.load_window()

        print("hej from trigger emit")
        for i in range(len(CommVariables.from_robot.position)):
            p = CommVariables.from_robot.position[i]
            self.sliders[i].act_rad.setText(str(round(p, 5)))

        
        

    def load_window(self):
        print("LOADING UI")
        print(CommVariables.poses)
        print(CommVariables.no_of_joints)
        grid = QGridLayout()

        self.labels = self.make_label_boxes()
        self.sliders = self.make_sliders(self.labels)



         # populate the grid with widgets:
        for i in range(len(self.sliders)):
            grid.addWidget(self.sliders[i].box, i, 0)

        for i in range(len(self.labels)):
            l = self.labels[i]
            grid.addWidget(l.box2, i, 1)
            grid.addWidget(l.box1, i, 2)
            grid.addWidget(l.box3, i, 3)


        # for line_box in self.line_boxes:
        #     grid.addWidget(line_box, self.line_boxes.index(line_box), 4)

        # grid.addWidget(self.speed_box, 6, 0, 1, 4)
        # grid.addWidget(self.pose_saver_box, 7, 0, 1, 4)
        # grid.addWidget(self.combo_box_box, 8, 0, 1, 4)
        # grid.addWidget(self.current_pose_box, 9, 0, 1, 2)
        # grid.addWidget(self.stop_box, 9, 2, 1, 1)
        # grid.addWidget(self.radio_1_box, 9, 3, 1, 1)

        

        self.setLayout(grid)
        self.setWindowTitle(CommVariables.node_name)
        self.resize(550, 250)

    def make_label_boxes(self):
        result = []
        for name in CommVariables.joint_limits.keys():
            label_box_1 = QGroupBox("ref_rad")
            ref_rad = QLabel("value")
            label_box_1_layout = QVBoxLayout()
            label_box_1_layout.addWidget(ref_rad)
            label_box_1.setLayout(label_box_1_layout)
            label_box_1.setMinimumWidth(90)
            label_box_1.setMaximumWidth(90)

            label_box_2 = QGroupBox("ref_deg")
            ref_deg = QLabel("value")
            label_box_2_layout = QVBoxLayout()
            label_box_2_layout.addWidget(ref_deg)
            label_box_2.setLayout(label_box_2_layout)
            label_box_2.setMinimumWidth(90)
            label_box_2.setMaximumWidth(90)

            label_box_3 = QGroupBox("act_rad")
            act_rad = QLabel("value")
            label_box_3_layout = QVBoxLayout()
            label_box_3_layout.addWidget(act_rad)
            label_box_3.setLayout(label_box_3_layout)
            label_box_3.setMinimumWidth(90)
            label_box_3.setMaximumWidth(90)
            label_box_3.setMinimumWidth(90)
            label_box_3.setMaximumWidth(90)

            result.append({
                "name": name,
                "box1": label_box_1,
                "ref_rad": ref_rad,
                "box2": label_box_2,
                "ref_deg": ref_deg,
                "box3": label_box_3,
                "act_deg": act_rad,
            })

        return result



    def make_sliders(self, labels):
        result = []
        for i, name in enumerate(CommVariables.joint_limits):
            limits = CommVariables.joint_limits[name]
            slider_box = QGroupBox(name)
            slider = QDoubleSlider(Qt.Horizontal)
            slider_box_layout = QVBoxLayout()
            slider.setFocusPolicy(Qt.StrongFocus)
            slider.setTickPosition(QSlider.TicksBothSides)
            slider.setTickInterval(5000)
            slider.setMinimum(limits[min])
            slider.setMaximum(limits[max])
            slider.setSingleStep(1)
            slider.setMinimumWidth(300)
            slider_box_layout.addWidget(slider)
            slider_box.setLayout(slider_box_layout)
            slider.setEnabled(False)

            slider.valueChanged.connect(lambda slider=slider, i=i: slider_change(name, slider, i))
            def slider_change(slider, i):
                l = labels[i]
                l.ref_rad.setText('{}'.format(round(slider.value() * math.pi / 180, 5)))
                l.ref_deg.setNum(slider.value())
                CommVariables.to_robot.gui_joint_control[i] = slider.value() * math.pi / 180
                CommVariables.trigger_node()

            result.append({"box": slider_box, "slider": slider})
        
        return result


class QDoubleSlider(QSlider):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.decimals = 5
        self._max_int = 10 ** self.decimals

        super().setMinimum(0)
        super().setMaximum(self._max_int)

        self._min_value = 0.0
        self._max_value = 1.0

    @property
    def _value_range(self):
        return self._max_value - self._min_value

    def value(self):
        return float(super().value()) / self._max_int * self._value_range + self._min_value

    def setValue(self, value):
        super().setValue(int((value - self._min_value) / self._value_range * self._max_int))

    def setMinimum(self, value):
        if value > self._max_value:
            raise ValueError("Minimum limit cannot be higher than maximum")

        self._min_value = value
        self.setValue(self.value())

    def setMaximum(self, value):
        if value < self._min_value:
            raise ValueError("Minimum limit cannot be higher than maximum")

        self._max_value = value
        self.setValue(self.value())

    def minimum(self):
        return self._min_value

    def maximum(self):
        return self._max_value



def main(args=None):

    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            gui = RobotGUI()
            rclpy.spin(gui)
            gui.destroy_node()
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
