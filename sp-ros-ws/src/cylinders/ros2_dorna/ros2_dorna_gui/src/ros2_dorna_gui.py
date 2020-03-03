import sys
import time
import ast
import csv
import os
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ros2_dorna_msgs.msg import DornaGuiToEsd
from ros2_dorna_msgs.msg import DornaEsdToGui
from ros2_dorna_msgs.msg import DornaGuiToUtils
from ros2_dorna_msgs.msg import DornaUtilsToGui
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class CommVariables():
    gui_control_enabled = False
    speed_slider_value = 0
    slider_1_value = 0
    slider_2_value = 0
    slider_3_value = 0
    slider_4_value = 0
    slider_5_value = 0
    slider_6_value = 0

    ref_1_value = 0.0
    ref_2_value = 0.0
    ref_3_value = 0.0
    ref_4_value = 0.0
    ref_5_value = 0.0
    ref_6_value = 0.0

    pose_name = ""
    actual_pose = ""
    actual_joint_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    saved_poses = []
    utility_action = ""
    def __init__(self, parent=None):
        super(CommVariables, self).__init__()

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

class Window(QWidget, CommVariables):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        self.namespace = ""

        if len(sys.argv) != 2:
            self.namespace = sys.argv[1]
        else:
            pass

        if self.namespace != "":
            self.joint_names = [self.namespace + "/" + "dorna_axis_1_joint",
                                self.namespace + "/" + "dorna_axis_2_joint",
                                self.namespace + "/" + "dorna_axis_3_joint",
                                self.namespace + "/" + "dorna_axis_4_joint",
                                self.namespace + "/" + "dorna_axis_5_joint"]
        else:
            self.joint_names = ["dorna_axis_1_joint",
                                "dorna_axis_2_joint",
                                "dorna_axis_3_joint",
                                "dorna_axis_4_joint",
                                "dorna_axis_5_joint"]

        self.joints_input = os.path.join(get_package_share_directory('ros2_dorna_utilities'),
            'poses', 'joint_poses.csv')

        minimum = -180
        maximum = 180
        step = 1

        grid = QGridLayout()

        # Sliders setup:
        self.slider_box_1 = QGroupBox(self.joint_names[0])
        self.slider_box_2 = QGroupBox(self.joint_names[1])
        self.slider_box_3 = QGroupBox(self.joint_names[2])
        self.slider_box_4 = QGroupBox(self.joint_names[3])
        self.slider_box_5 = QGroupBox(self.joint_names[4])
        # self.slider_box_6 = QGroupBox(self.joint_names[5])

        self.slider_boxes = [self.slider_box_1, self.slider_box_2, self.slider_box_3, self.slider_box_4, self.slider_box_5]

        self.slider_1 = QDoubleSlider(Qt.Horizontal)
        self.slider_2 = QDoubleSlider(Qt.Horizontal)
        self.slider_3 = QDoubleSlider(Qt.Horizontal)
        self.slider_4 = QDoubleSlider(Qt.Horizontal)
        self.slider_5 = QDoubleSlider(Qt.Horizontal)
        # self.slider_6 = QDoubleSlider(Qt.Horizontal)

        self.sliders = [self.slider_1, self.slider_2, self.slider_3, self.slider_4, self.slider_5]

        self.slider_box_1_layout = QVBoxLayout()
        self.slider_box_2_layout = QVBoxLayout()
        self.slider_box_3_layout = QVBoxLayout()
        self.slider_box_4_layout = QVBoxLayout()
        self.slider_box_5_layout = QVBoxLayout()
        # self.slider_box_6_layout = QVBoxLayout()

        for slider in self.sliders:
            slider.setFocusPolicy(Qt.StrongFocus)
            slider.setTickPosition(QSlider.TicksBothSides)
            slider.setTickInterval(5000)
            slider.setMinimum(minimum)
            slider.setMaximum(maximum)
            slider.setSingleStep(step)
            slider.setMinimumWidth(300)

        self.slider_box_1_layout.addWidget(self.slider_1)
        self.slider_box_2_layout.addWidget(self.slider_2)
        self.slider_box_3_layout.addWidget(self.slider_3)
        self.slider_box_4_layout.addWidget(self.slider_4)
        self.slider_box_5_layout.addWidget(self.slider_5)
        # self.slider_box_6_layout.addWidget(self.slider_6)

        self.slider_box_1.setLayout(self.slider_box_1_layout)
        self.slider_box_2.setLayout(self.slider_box_2_layout)
        self.slider_box_3.setLayout(self.slider_box_3_layout)
        self.slider_box_4.setLayout(self.slider_box_4_layout)
        self.slider_box_5.setLayout(self.slider_box_5_layout)
        # self.slider_box_6.setLayout(self.slider_box_6_layout)


        # Indicator labels setup:
        self.label_box_1 = QGroupBox("ref_rad")
        self.label_box_2 = QGroupBox("ref_rad")
        self.label_box_3 = QGroupBox("ref_rad")
        self.label_box_4 = QGroupBox("ref_rad")
        self.label_box_5 = QGroupBox("ref_rad")
        # self.label_box_6 = QGroupBox("ref_rad")

        # Indicator labels setup:
        self.label_box_12 = QGroupBox("ref_deg")
        self.label_box_22 = QGroupBox("ref_deg")
        self.label_box_32 = QGroupBox("ref_deg")
        self.label_box_42 = QGroupBox("ref_deg")
        self.label_box_52 = QGroupBox("ref_deg")
        # self.label_box_62 = QGroupBox("ref_deg")

         # Indicator labels setup:
        self.label_box_13 = QGroupBox("act_rad")
        self.label_box_23 = QGroupBox("act_rad")
        self.label_box_33 = QGroupBox("act_rad")
        self.label_box_43 = QGroupBox("act_rad")
        self.label_box_53 = QGroupBox("act_rad")
        # self.label_box_63 = QGroupBox("act_rad")

        self.label_boxes = [self.label_box_1, self.label_box_2, self.label_box_3, self.label_box_4, self.label_box_5]
        self.label_boxes2 = [self.label_box_12, self.label_box_22, self.label_box_32, self.label_box_42, self.label_box_52]
        self.label_boxes3 = [self.label_box_13, self.label_box_23, self.label_box_33, self.label_box_43, self.label_box_53]

        for label_box in self.label_boxes:
            label_box.setMinimumWidth(90)
            label_box.setMaximumWidth(90)

        for label_box in self.label_boxes2:
            label_box.setMinimumWidth(90)
            label_box.setMaximumWidth(90)

        for label_box in self.label_boxes3:
            label_box.setMinimumWidth(90)
            label_box.setMaximumWidth(90)

        self.label_1 = QLabel("value")
        self.label_2 = QLabel("value")
        self.label_3 = QLabel("value")
        self.label_4 = QLabel("value")
        self.label_5 = QLabel("value")
        # self.label_6 = QLabel("value")

        self.label_12 = QLabel("value")
        self.label_22 = QLabel("value")
        self.label_32 = QLabel("value")
        self.label_42 = QLabel("value")
        self.label_52 = QLabel("value")
        # self.label_62 = QLabel("value")

        self.label_13 = QLabel("value")
        self.label_23 = QLabel("value")
        self.label_33 = QLabel("value")
        self.label_43 = QLabel("value")
        self.label_53 = QLabel("value")
        # self.label_63 = QLabel("value")

        self.labels = [self.label_1, self.label_2, self.label_3, self.label_4, self.label_5]
        self.labels2 = [self.label_12, self.label_22, self.label_32, self.label_42, self.label_52]
        self.labels3 = [self.label_13, self.label_23, self.label_33, self.label_43, self.label_53]

        self.label_box_1_layout = QVBoxLayout()
        self.label_box_2_layout = QVBoxLayout()
        self.label_box_3_layout = QVBoxLayout()
        self.label_box_4_layout = QVBoxLayout()
        self.label_box_5_layout = QVBoxLayout()
        # self.label_box_6_layout = QVBoxLayout()

        self.label_box_12_layout = QVBoxLayout()
        self.label_box_22_layout = QVBoxLayout()
        self.label_box_32_layout = QVBoxLayout()
        self.label_box_42_layout = QVBoxLayout()
        self.label_box_52_layout = QVBoxLayout()
        # self.label_box_62_layout = QVBoxLayout()

        self.label_box_13_layout = QVBoxLayout()
        self.label_box_23_layout = QVBoxLayout()
        self.label_box_33_layout = QVBoxLayout()
        self.label_box_43_layout = QVBoxLayout()
        self.label_box_53_layout = QVBoxLayout()
        # self.label_box_63_layout = QVBoxLayout()

        self.label_box_1_layout.addWidget(self.label_1)
        self.label_box_2_layout.addWidget(self.label_2)
        self.label_box_3_layout.addWidget(self.label_3)
        self.label_box_4_layout.addWidget(self.label_4)
        self.label_box_5_layout.addWidget(self.label_5)
        # self.label_box_6_layout.addWidget(self.label_6)

        self.label_box_12_layout.addWidget(self.label_12)
        self.label_box_22_layout.addWidget(self.label_22)
        self.label_box_32_layout.addWidget(self.label_32)
        self.label_box_42_layout.addWidget(self.label_42)
        self.label_box_52_layout.addWidget(self.label_52)
        # self.label_box_62_layout.addWidget(self.label_62)

        self.label_box_13_layout.addWidget(self.label_13)
        self.label_box_23_layout.addWidget(self.label_23)
        self.label_box_33_layout.addWidget(self.label_33)
        self.label_box_43_layout.addWidget(self.label_43)
        self.label_box_53_layout.addWidget(self.label_53)
        # self.label_box_63_layout.addWidget(self.label_63)

        self.label_box_1.setLayout(self.label_box_1_layout)
        self.label_box_2.setLayout(self.label_box_2_layout)
        self.label_box_3.setLayout(self.label_box_3_layout)
        self.label_box_4.setLayout(self.label_box_4_layout)
        self.label_box_5.setLayout(self.label_box_5_layout)
        # self.label_box_6.setLayout(self.label_box_6_layout)

        self.label_box_12.setLayout(self.label_box_12_layout)
        self.label_box_22.setLayout(self.label_box_22_layout)
        self.label_box_32.setLayout(self.label_box_32_layout)
        self.label_box_42.setLayout(self.label_box_42_layout)
        self.label_box_52.setLayout(self.label_box_52_layout)
        # self.label_box_62.setLayout(self.label_box_62_layout)

        self.label_box_13.setLayout(self.label_box_13_layout)
        self.label_box_23.setLayout(self.label_box_23_layout)
        self.label_box_33.setLayout(self.label_box_33_layout)
        self.label_box_43.setLayout(self.label_box_43_layout)
        self.label_box_53.setLayout(self.label_box_53_layout)
        # self.label_box_63.setLayout(self.label_box_63_layout)

        # Ref pos commander setup
        self.line_box_1 = QGroupBox("set_ref_deg")
        self.line_box_2 = QGroupBox("set_ref_deg")
        self.line_box_3 = QGroupBox("set_ref_deg")
        self.line_box_4 = QGroupBox("set_ref_deg")
        self.line_box_5 = QGroupBox("set_ref_deg")
        # self.line_box_6 = QGroupBox("set_ref_deg")

        self.line_boxes = [self.line_box_1, self.line_box_2, self.line_box_3, self.line_box_4, self.line_box_5]

        self.line_1 = QLineEdit()
        self.line_2 = QLineEdit()
        self.line_3 = QLineEdit()
        self.line_4 = QLineEdit()
        self.line_5 = QLineEdit()
        # self.line_6 = QLineEdit()

        self.lines = [self.line_1, self.line_2, self.line_3, self.line_4, self.line_5]

        self.line_1.setMaximumWidth(120)
        self.line_2.setMaximumWidth(120)
        self.line_3.setMaximumWidth(120)
        self.line_4.setMaximumWidth(120)
        self.line_5.setMaximumWidth(120)
        # self.line_6.setMaximumWidth(120)

        self.button_1 = QPushButton('set')
        self.button_2 = QPushButton('set')
        self.button_3 = QPushButton('set')
        self.button_4 = QPushButton('set')
        self.button_5 = QPushButton('set')
        # self.button_6 = QPushButton('set')

        self.buttons = [self.button_1, self.button_2, self.button_3, self.button_4, self.button_5]

        self.line_box_1_layout = QHBoxLayout()
        self.line_box_2_layout = QHBoxLayout()
        self.line_box_3_layout = QHBoxLayout()
        self.line_box_4_layout = QHBoxLayout()
        self.line_box_5_layout = QHBoxLayout()
        # self.line_box_6_layout = QHBoxLayout()

        self.line_box_1_layout.addWidget(self.line_1)
        self.line_box_1_layout.addWidget(self.button_1)
        self.line_box_2_layout.addWidget(self.line_2)
        self.line_box_2_layout.addWidget(self.button_2)
        self.line_box_3_layout.addWidget(self.line_3)
        self.line_box_3_layout.addWidget(self.button_3)
        self.line_box_4_layout.addWidget(self.line_4)
        self.line_box_4_layout.addWidget(self.button_4)
        self.line_box_5_layout.addWidget(self.line_5)
        self.line_box_5_layout.addWidget(self.button_5)
        # self.line_box_6_layout.addWidget(self.line_6)
        # self.line_box_6_layout.addWidget(self.button_6)

        self.line_box_1.setLayout(self.line_box_1_layout)
        self.line_box_2.setLayout(self.line_box_2_layout)
        self.line_box_3.setLayout(self.line_box_3_layout)
        self.line_box_4.setLayout(self.line_box_4_layout)
        self.line_box_5.setLayout(self.line_box_5_layout)
        # self.line_box_6.setLayout(self.line_box_6_layout)


        # One point of control:
        for slider in self.sliders:
            slider.setEnabled(False)

        for label in self.labels:
            label.setEnabled(False)

        for line in self.lines:
            line.setEnabled(False)

        for label in self.labels2:
            label.setEnabled(False)

        for button in self.buttons:
            button.setEnabled(False)

        self.radio_1_box = QGroupBox("gui_control")
        self.radio_1_box_layout = QHBoxLayout()
        self.radio_1 = QRadioButton("enabled")
        self.radio_1.setChecked(False)
        self.radio_1_box_layout.addWidget(self.radio_1)
        self.radio_1_box.setLayout(self.radio_1_box_layout)

        def radio_state():
            if self.radio_1.isChecked() == True:
                CommVariables.gui_control_enabled = True
                for slider in self.sliders:
                    slider.setEnabled(True)

                for label in self.labels:
                    label.setEnabled(True)

                for label in self.labels2:
                    label.setEnabled(True)

                for line in self.lines:
                    line.setEnabled(True)

                for button in self.buttons:
                    button.setEnabled(True)

                self.speed_box.setEnabled(True)
                self.pose_saver_box.setEnabled(True)
                self.combo_box_box.setEnabled(True)
                self.current_pose_box.setEnabled(True)

            else:
                CommVariables.gui_control_enabled = False
                for slider in self.sliders:
                    slider.setEnabled(False)

                for label in self.labels:
                    label.setEnabled(False)

                for label in self.labels2:
                    label.setEnabled(False)

                for line in self.lines:
                    line.setEnabled(False)

                for button in self.buttons:
                    button.setEnabled(False)

                self.speed_box.setEnabled(False)
                self.pose_saver_box.setEnabled(False)
                self.combo_box_box.setEnabled(False)
                self.current_pose_box.setEnabled(False)

        self.radio_1.toggled.connect(radio_state)

        def slider_1_change():
            self.label_1.setText('{}'.format(round(self.slider_1.value() * math.pi / 180, 5)))
            self.label_12.setNum(self.slider_1.value())
            CommVariables.slider_1_value = self.slider_1.value()

        def slider_2_change():
            self.label_2.setText('{}'.format(round(self.slider_2.value() * math.pi / 180, 5)))
            self.label_22.setNum(self.slider_2.value())
            CommVariables.slider_2_value = self.slider_2.value()

        def slider_3_change():
            self.label_3.setText('{}'.format(round(self.slider_3.value() * math.pi / 180, 5)))
            self.label_32.setNum(self.slider_3.value())
            CommVariables.slider_3_value = self.slider_3.value()

        def slider_4_change():
            self.label_4.setText('{}'.format(round(self.slider_4.value() * math.pi / 180, 5)))
            self.label_42.setNum(self.slider_4.value())
            CommVariables.slider_4_value = self.slider_4.value()

        def slider_5_change():
            self.label_5.setText('{}'.format(round(self.slider_5.value() * math.pi / 180, 5)))
            self.label_52.setNum(self.slider_5.value())
            CommVariables.slider_5_value = self.slider_5.value()

        # def slider_6_change():
        #     self.label_6.setText('{}'.format(round(self.slider_6.value() * math.pi / 180, 5)))
        #     self.label_62.setNum(self.slider_6.value())
        #     CommVariables.slider_6_value = self.slider_6.value()

        # inter-widget communications:
        self.slider_1.valueChanged.connect(slider_1_change)
        self.slider_2.valueChanged.connect(slider_2_change)
        self.slider_3.valueChanged.connect(slider_3_change)
        self.slider_4.valueChanged.connect(slider_4_change)
        self.slider_5.valueChanged.connect(slider_5_change)
        # self.slider_6.valueChanged.connect(slider_6_change)

        CommVariables.ref_1_value = self.label_12.text()

        def button_1_clicked():
            self.slider_1.setValue(float(self.line_1.text()))

        def button_2_clicked():
            self.slider_2.setValue(int(self.line_2.text()))

        def button_3_clicked():
            self.slider_3.setValue(int(self.line_3.text()))

        def button_4_clicked():
            self.slider_4.setValue(int(self.line_4.text()))

        def button_5_clicked():
            self.slider_5.setValue(int(self.line_5.text()))

        # def button_6_clicked():
        #     self.slider_6.setValue(int(self.line_6.text()))

        self.button_1.clicked.connect(button_1_clicked)
        self.button_2.clicked.connect(button_2_clicked)
        self.button_3.clicked.connect(button_3_clicked)
        self.button_4.clicked.connect(button_4_clicked)
        self.button_5.clicked.connect(button_5_clicked)
        # self.button_6.clicked.connect(button_6_clicked)

        self.slider_1_value = self.slider_1.value()
        self.slider_2_value = self.slider_2.value()
        self.slider_3_value = self.slider_3.value()
        self.slider_4_value = self.slider_4.value()
        self.slider_5_value = self.slider_5.value()
        # self.slider_6_value = self.slider_6.value()

        def update_labels():
            self.label_13.setText(str(round(self.actual_joint_pose[0], 5)))
            self.label_23.setText(str(round(self.actual_joint_pose[1], 5)))
            self.label_33.setText(str(round(self.actual_joint_pose[2], 5)))
            self.label_43.setText(str(round(self.actual_joint_pose[3], 5)))
            self.label_53.setText(str(round(self.actual_joint_pose[4], 5)))
            # self.label_63.setText(str(round(self.actual_joint_pose[5], 5)))

        self.timer = QTimer()
        self.timer.timeout.connect(update_labels)
        self.timer.start(100) # repeat self.update_labelTime every 0.01 sec

        # speedwidget:
        self.speed_box = QGroupBox("robot_speed")
        self.speed_box_layout = QHBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setFocusPolicy(Qt.StrongFocus)
        self.speed_slider.setTickPosition(QSlider.TicksBothSides)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setSingleStep(step)
        self.speed_slider.setMinimumWidth(200)
        self.speed_line = QLineEdit("%")
        self.speed_line.setMaximumWidth(80)
        self.speed_button = QPushButton("set")
        self.speed_box_layout.addWidget(self.speed_slider)
        self.speed_box_layout.addWidget(self.speed_line)
        self.speed_box_layout.addWidget(self.speed_button)
        self.speed_box.setLayout(self.speed_box_layout)
        self.speed_box.setEnabled(False)

        def speed_slider_change():
            CommVariables.speed_slider_value = self.speed_slider.value()

        def speed_button_clicked():
            self.speed_slider.setValue(int(self.speed_line.text()))

        self.speed_button.clicked.connect(speed_button_clicked)
        self.speed_slider_value = self.speed_slider.value()
        self.speed_slider.valueChanged.connect(speed_slider_change)

        # pose saver widget:
        self.pose_saver_box = QGroupBox("pose_saver")
        self.pose_saver_box_layout = QHBoxLayout()
        self.pose_saver_label = QLabel("pose_name")
        self.pose_saver_line = QLineEdit("some_pose_name")
        self.pose_saver_update_button = QPushButton("update")
        self.pose_saver_delete_button = QPushButton("delete")
        self.pose_saver_box_layout.addWidget(self.pose_saver_label)
        self.pose_saver_box_layout.addWidget(self.pose_saver_line)
        self.pose_saver_box_layout.addWidget(self.pose_saver_update_button)
        self.pose_saver_box_layout.addWidget(self.pose_saver_delete_button)
        self.pose_saver_box.setLayout(self.pose_saver_box_layout)
        self.pose_saver_box.setEnabled(False)

        def pose_saver_update_button_clicked():
            CommVariables.pose_name = self.pose_saver_line.text()
            CommVariables.utility_action = "update"

        self.pose_saver_update_button.clicked.connect(pose_saver_update_button_clicked)

        def pose_saver_delete_button_clicked():
            CommVariables.pose_name = self.pose_saver_line.text()
            CommVariables.utility_action = "delete"

        self.pose_saver_delete_button.clicked.connect(pose_saver_delete_button_clicked)

        # goto pose widget:
        self.combo_box_box = QGroupBox("go_to_pose")
        self.combo_box_box_layout = QHBoxLayout()
        self.combo_box_label = QLabel("pose_name")
        self.combo_box = QComboBox()
        self.combo_box.setMinimumWidth(400)
        self.combo_box_button = QPushButton("go")
        self.combo_box_button.setMaximumWidth(80)
        self.combo_box_box_layout.addWidget(self.combo_box_label)
        self.combo_box_box_layout.addWidget(self.combo_box)
        self.combo_box_box_layout.addWidget(self.combo_box_button)
        self.combo_box_box.setLayout(self.combo_box_box_layout)
        self.combo_box_box.setEnabled(False)

        self.old_saved_poses = CommVariables.saved_poses

        def refresh_combo_box():
            if self.old_saved_poses != CommVariables.saved_poses:
                self.combo_box.clear()
                self.combo_box.addItems(CommVariables.saved_poses)
                self.old_saved_poses = CommVariables.saved_poses
            else:
                pass

        self.timer2 = QTimer()
        self.timer2.timeout.connect(refresh_combo_box)
        self.timer2.start(500) # repeat self.update_labelTime every 0.5 sec

        def get_pose_from_pose_name(name):
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

        def combo_box_button_clicked():
            pose = get_pose_from_pose_name(self.combo_box.currentText())
            self.slider_1.setValue(180*pose[0]/math.pi)
            self.slider_2.setValue(180*pose[1]/math.pi)
            self.slider_3.setValue(180*pose[2]/math.pi)
            self.slider_4.setValue(180*pose[3]/math.pi)
            self.slider_5.setValue(180*pose[4]/math.pi)
            # self.slider_6.setValue(180*pose[5]/math.pi)
            # print(pose)

        self.combo_box_button.clicked.connect(combo_box_button_clicked)

        # current pose widget:
        self.current_pose_box = QGroupBox("current_pose")
        self.current_pose_box_layout = QHBoxLayout()
        self.current_pose_label = QLabel("")
        self.current_pose_box_layout.addWidget(self.current_pose_label)
        self.current_pose_box.setLayout(self.current_pose_box_layout)
        self.current_pose_box.setEnabled(False)

        def update_current_pose_label():
            self.current_pose_label.setText(str(CommVariables.actual_pose))

        self.timer3 = QTimer()
        self.timer3.timeout.connect(update_current_pose_label)
        self.timer3.start(100) # repeat self.update_labelTime every 0.1 sec

        # update sliders:
        def update_sliders():
            if CommVariables.gui_control_enabled == False:
                self.slider_1.setValue(180*CommVariables.actual_joint_pose[0]/math.pi)
                self.slider_2.setValue(180*CommVariables.actual_joint_pose[1]/math.pi)
                self.slider_3.setValue(180*CommVariables.actual_joint_pose[2]/math.pi)
                self.slider_4.setValue(180*CommVariables.actual_joint_pose[3]/math.pi)
                self.slider_5.setValue(180*CommVariables.actual_joint_pose[4]/math.pi)
                # self.slider_6.setValue(180*CommVariables.actual_joint_pose[5]/math.pi)
            else:
                pass

        self.timer4 = QTimer()
        self.timer4.timeout.connect(update_sliders)
        self.timer4.start(10) # repeat self.update_labelTime every 0.1 sec


        # populate the grid with widgets:
        for slider_box in self.slider_boxes:
            grid.addWidget(slider_box, self.slider_boxes.index(slider_box), 0)

        for label_box in self.label_boxes2:
            grid.addWidget(label_box, self.label_boxes2.index(label_box), 1)

        for label_box in self.label_boxes:
            grid.addWidget(label_box, self.label_boxes.index(label_box), 2)

        for label_box in self.label_boxes3:
            grid.addWidget(label_box, self.label_boxes3.index(label_box), 3)

        for line_box in self.line_boxes:
            grid.addWidget(line_box, self.line_boxes.index(line_box), 4)

        grid.addWidget(self.speed_box, 6, 0, 1, 4)
        grid.addWidget(self.pose_saver_box, 7, 0, 1, 4)
        grid.addWidget(self.combo_box_box, 8, 0, 1, 4)
        grid.addWidget(self.current_pose_box, 9, 0, 1, 1)
        grid.addWidget(self.radio_1_box, 9, 1, 1, 3)

        self.setLayout(grid)

        if self.namespace == '':
            self.setWindowTitle("Dorna joint pose controller")
        else:
            self.setWindowTitle("Dorna joint pose controller - " + self.namespace)
        self.resize(550, 250)

class Ros2DornaGui(Node, CommVariables):

    def __init__(self):
        super().__init__("ros2_dorna_gui")

        self.namespace = ""

        if len(sys.argv) != 2:
            self.namespace = sys.argv[1]
        else:
            pass

        self.gui_to_esd_msg = DornaGuiToEsd()
        self.gui_to_utils_msg = DornaGuiToUtils()
        self.joint_state = JointState()

        self.gui_to_esd_msg.gui_control_enabled = False
        self.gui_to_esd_msg.gui_speed_control = 0
        self.gui_to_esd_msg.gui_joint_control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.actual_pose = ""
        self.actual_joint_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.saved_poses = []

        self.gui_to_esd_timer_period = 0.5
        self.gui_to_utils_timer_period = 0.2

        if self.namespace != "":
            self.joint_names = [self.namespace + "/" + "dorna_axis_1_joint",
                                self.namespace + "/" + "dorna_axis_2_joint",
                                self.namespace + "/" + "dorna_axis_3_joint",
                                self.namespace + "/" + "dorna_axis_4_joint",
                                self.namespace + "/" + "dorna_axis_5_joint"]
        else:
            self.joint_names = ["dorna_axis_1_joint",
                                "dorna_axis_2_joint",
                                "dorna_axis_3_joint",
                                "dorna_axis_4_joint",
                                "dorna_axis_5_joint"]

        self.utils_to_gui_subscriber = self.create_subscription(
            DornaUtilsToGui,
            "/dorna_utils_to_gui",
            self.utils_to_gui_callback,
            10)

        self.esd_to_gui_subscriber = self.create_subscription(
            DornaEsdToGui,
            "/dorna_esd_to_gui",
            self.esd_to_gui_callback,
            10)

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/dorna_joint_states",
            self.joint_state_callback,
            10)

        time.sleep(2)

        self.gui_to_esd_publisher_ = self.create_publisher(
            DornaGuiToEsd,
            "/dorna_gui_to_esd",
            10)

        self.gui_to_utils_publisher_ = self.create_publisher(
            DornaGuiToUtils,
            "/dorna_gui_to_utils",
            10)

        self.gui_to_esd_timer = self.create_timer(
            self.gui_to_esd_timer_period,
            self.gui_to_esd_callback)

        self.gui_to_utils_timer = self.create_timer(
            self.gui_to_utils_timer_period,
            self.gui_to_utils_callback)

    def gui_to_esd_callback(self):
        self.gui_to_esd_msg.gui_control_enabled = CommVariables.gui_control_enabled

        if CommVariables.speed_slider_value >= 0 and CommVariables.speed_slider_value <= 100:
            self.gui_to_esd_msg.gui_speed_control = CommVariables.speed_slider_value
        else:
            pass

        self.gui_to_esd_msg.gui_joint_control[0] = CommVariables.slider_1_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[1] = CommVariables.slider_2_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[2] = CommVariables.slider_3_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[3] = CommVariables.slider_4_value * math.pi / 180
        self.gui_to_esd_msg.gui_joint_control[4] = CommVariables.slider_5_value * math.pi / 180
        # self.gui_to_esd_msg.gui_joint_control[5] = CommVariables.slider_6_value * math.pi / 180

        self.gui_to_esd_publisher_.publish(self.gui_to_esd_msg)

    def gui_to_utils_callback(self):
        self.gui_to_utils_msg.utility_action = CommVariables.utility_action
        self.gui_to_utils_msg.utility_pose_name = CommVariables.pose_name
        self.gui_to_utils_publisher_.publish(self.gui_to_utils_msg)

    def esd_to_gui_callback(self, data):
        self.actual_pose = data.actual_pose
        CommVariables.actual_pose = self.actual_pose

    def utils_to_gui_callback(self, data):
        self.saved_poses = data.saved_poses
        CommVariables.saved_poses = self.saved_poses

    def joint_state_callback(self, data):
        self.actual_joint_pose = data.position
        CommVariables.actual_joint_pose = self.actual_joint_pose

def main(args=None):

    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            ros2_dorna_gui = Ros2DornaGui()
            rclpy.spin(ros2_dorna_gui)
            ros2_dorna_gui.destroy_node()
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
