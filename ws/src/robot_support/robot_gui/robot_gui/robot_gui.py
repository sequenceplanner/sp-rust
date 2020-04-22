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

    to_robot.gui_control_enabled = True
    to_robot.gui_speed_control = 50
    to_robot.gui_joint_control = []

    node_name = ""
    
    saved_poses_file = ""
    joint_names = []
    joint_limit_max = []
    joint_limit_min = []

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
            node_name= "robot_gui"
        )

        CommVariables.trigger_node = self.trigger

        CommVariables.node_name = self.get_name()
        CommVariables.saved_poses_file = self.declare_parameter("saved_poses_file").value
        CommVariables.joint_names = self.declare_parameter("joint_names").value
        CommVariables.no_of_joints = len(CommVariables.joint_names)

        # for some reason these parameters becomes ([...]) so we need to [0] when using
        CommVariables.joint_limit_max = self.declare_parameter("joint_limit_max", value=[180]*self.no_of_joints).value,
        CommVariables.joint_limit_min = self.declare_parameter("joint_limit_min", value=[-180]*self.no_of_joints).value,
        
        CommVariables.joint_tolerance = self.declare_parameter("joint_tolerance", value=0.01).value

        CommVariables.poses = load_poses(CommVariables.saved_poses_file)
        CommVariables.to_robot.gui_joint_control = [0.0]*self.no_of_joints

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

        for i in range(len(CommVariables.from_robot.position)):
            p = CommVariables.from_robot.position[i]
            self.labels[i]['act_rad'].setText(str(round(p, 5)))

        
        

    def load_window(self):
        print("LOADING UI")
        grid = QGridLayout()

        self.labels = self.make_label_boxes()
        self.sliders = self.make_sliders(self.labels)
        self.set_boxes = self.make_set_boxes(self.sliders)
        self.gui_control_box = self.make_gui_control_box(self.labels, self.sliders, self.set_boxes)
        self.estop_box = self.make_estop_box(self.sliders)


         # populate the grid with widgets:
        for i, s in enumerate(self.sliders):
            grid.addWidget(s['box'], i, 0)

        for i, l in enumerate(self.labels):
            grid.addWidget(l['box2'], i, 1)
            grid.addWidget(l['box1'], i, 2)
            grid.addWidget(l['box3'], i, 3)

        for i, sb in enumerate(self.set_boxes):
            grid.addWidget(sb['box'], i, 4)


        # grid.addWidget(self.speed_box, 6, 0, 1, 4)
        # grid.addWidget(self.pose_saver_box, 7, 0, 1, 4)
        # grid.addWidget(self.combo_box_box, 8, 0, 1, 4)
        # grid.addWidget(self.current_pose_box, 9, 0, 1, 2)
        grid.addWidget(self.estop_box, 9, 2, 1, 1)
        grid.addWidget(self.gui_control_box, 9, 3, 1, 1)

        self.trigger_enabled(self.labels, self.sliders, self.set_boxes)
        

        self.setLayout(grid)
        self.setWindowTitle(CommVariables.node_name)
        self.resize(600, 250)

    def trigger_enabled(self, lables, sliders, set_box):
        for l in lables:
            l['ref_rad'].setEnabled(CommVariables.to_robot.gui_control_enabled)
            l['ref_deg'].setEnabled(CommVariables.to_robot.gui_control_enabled)

        for s in sliders:
            s['slider'].setEnabled(CommVariables.to_robot.gui_control_enabled)

        for x in set_box:
            x['button'].setEnabled(CommVariables.to_robot.gui_control_enabled)

    def make_label_boxes(self):
        result = []
        for name in CommVariables.joint_names:
            label_box_1 = QGroupBox("ref_rad")
            ref_rad = QLabel('0.0')
            label_box_1_layout = QVBoxLayout()
            label_box_1_layout.addWidget(ref_rad)
            label_box_1.setLayout(label_box_1_layout)
            label_box_1.setMinimumWidth(90)
            label_box_1.setMaximumWidth(90)

            label_box_2 = QGroupBox("ref_deg")
            ref_deg = QLabel('0')
            label_box_2_layout = QVBoxLayout()
            label_box_2_layout.addWidget(ref_deg)
            label_box_2.setLayout(label_box_2_layout)
            label_box_2.setMinimumWidth(90)
            label_box_2.setMaximumWidth(90)

            label_box_3 = QGroupBox("act_rad")
            act_rad = QLabel('0.0')
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
                "act_rad": act_rad,
            })

        return result



    def make_sliders(self, labels):
        result = []
        for i, name in enumerate(CommVariables.joint_names):
            max_limit = CommVariables.joint_limit_max[0][i]
            min_limit = CommVariables.joint_limit_min[0][i]

            slider_box = QGroupBox(name)
            slider = QSlider(Qt.Horizontal)
            slider_box_layout = QVBoxLayout()
            slider.setFocusPolicy(Qt.StrongFocus)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(5000)
            slider.setMinimum(min_limit)
            slider.setMaximum(max_limit)
            slider.setSingleStep(1)
            slider.setMinimumWidth(300)
            slider_box_layout.addWidget(slider)
            slider_box.setLayout(slider_box_layout)
            slider.setValue(0.0)

            def slider_change(value, joint_no):
                l = labels[joint_no]
                l['ref_rad'].setText('{}'.format(round(value * math.pi / 180, 5)))
                l['ref_deg'].setNum(value)
                CommVariables.to_robot.gui_joint_control[joint_no] = value * math.pi / 180
                CommVariables.trigger_node()

            slider.valueChanged.connect(lambda value=slider.value(), joint_no=i: slider_change(value, joint_no))
            result.append({"box": slider_box, "slider": slider})

        return result

    def make_set_boxes(self, sliders):
        result = []
        for i in range(CommVariables.no_of_joints):
            line_box = QGroupBox("set_ref_deg")
            line = QLineEdit()
            line.setMinimumWidth(50)
            button = QPushButton('set')
            line_box_layout = QHBoxLayout()
            line_box_layout.addWidget(line)
            line_box_layout.addWidget(button)
            line_box.setLayout(line_box_layout)

            def clicked(i, set_box):
                sliders[i]['slider'].setValue(int(set_box.text()))
            button.clicked.connect(lambda i=i, set_box=line: clicked(i, set_box))

            result.append({"box": line_box, "button": button, "edit": line})
        return result

    def make_gui_control_box(self, lables, sliders, set_boxes):
        radio_box = QGroupBox("gui_control")
        radio_box_layout = QHBoxLayout()
        radio = QRadioButton("enabled")
        radio.setChecked(CommVariables.to_robot.gui_control_enabled)
        radio_box_layout.addWidget(radio)
        radio_box.setLayout(radio_box_layout)

        def toggle():
            CommVariables.to_robot.gui_control_enabled = radio.isChecked()
            self.trigger_enabled(lables, sliders, set_boxes)
        radio.toggled.connect(toggle)
        return radio_box

    def make_estop_box(self, sliders):
        stop_box = QGroupBox("emergency_stop")
        stop_box_layout = QHBoxLayout()
        stop_button = QPushButton("STOP")
        stop_button.setMaximumWidth(80)
        stop_box_layout.addWidget(stop_button)
        stop_box.setLayout(stop_box_layout)

        def stop_button_clicked():
            print('EMERGENCY STOP')
            CommVariables.to_robot.gui_control_enabled = True
            for i, j in enumerate(CommVariables.from_robot.position):
                CommVariables.to_robot.gui_joint_control[i] = j

            for i, s in enumerate(sliders):
                s['slider'].setValue(180*CommVariables.to_robot.gui_joint_control[i]/math.pi)

        stop_button.clicked.connect(stop_button_clicked)
        return stop_box


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
