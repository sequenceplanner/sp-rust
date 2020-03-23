import sys
import time
import ast
import csv
import os
import math
import threading
import json
from itertools import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sp_messages.msg import RunnerInfo
from sp_messages.msg import RunnerCommand

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks():
    info = RunnerInfo()
    cmd = RunnerCommand()

    trigger_node = None
    trigger_ui = None

    def __init__(self):
        super(Callbacks, self).__init__()


class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "sp_ui")
        Callbacks.__init__(self)

        Callbacks.trigger_node = self.trigger

        self.subscriber = self.create_subscription(
            RunnerInfo,
            "sp/runner/info",
            self.sp_cmd_callback,
            10)

        self.state_publisher = self.create_publisher(
            RunnerCommand,
            "sp/runner/command",
            10)

        print('Up and running...')
    
    def trigger(self):
        print("trigger node")
        #self.state_publisher.publish(Callbacks.cmd)
    
    def sp_cmd_callback(self, data):
        Callbacks.info = data
        #self.get_logger().info('info: "%s"' % data)
        if Callbacks.trigger_ui:
            Callbacks.trigger_ui()

        #self.state_publisher.publish(Callbacks.cmd)


class Window(QWidget, Callbacks):
    triggerSignal = pyqtSignal()

    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)
        #self.count = 0 

        self.state_model = QStandardItemModel(self)
        self.state_model_proxy = QSortFilterProxyModel(self)

        Callbacks.trigger_ui = self.trigger
        self.triggerSignal.connect(self.update_state_variables)
        self.state_map = {}

        grid = QGridLayout(self)
        grid.addWidget(self.info_widget())
        grid.addWidget(self.tree_widget())
        self.setLayout(grid)
        self.setWindowTitle("Sequence Planner Control UI")
        self.resize(600, 800)

        self.init = True

    def split_path(self, s):
        path = s.split("/")
        res = []
        if len(path) > 0:
            aggr = path[0]
            res = [(None, aggr)]
            for p in path[1:]:
                x = aggr
                aggr = aggr + "/" + p
                res.append((x, aggr))

        return res

    def get_leaf_name(self, path):
        return path.split("/")[-1]

    def get_parent(self, path):
        if path not in self.state_map:
            return self.state_model.invisibleRootItem()
        else:
            p = self.state_map[path]
            return self.state_model.itemFromIndex(p)

    def insert_parent(self, parent, name):
        if name not in self.state_map:
            item = QStandardItem()
            n = self.get_leaf_name(name)
            item.setData(n, Qt.DisplayRole)
            item.setData(name, Qt.ToolTipRole)
            parent.appendRow([item])
            self.state_map[name] = item.index()

    def insert_variable(self, parent, name, value):
        if name not in self.state_map:
            item = QStandardItem()
            n = self.get_leaf_name(name)
            item.setData(n, Qt.DisplayRole)
            item.setData(name, Qt.ToolTipRole)
            value = QStandardItem()
            value.setData(value, Qt.DisplayRole)
            value.setData(value, Qt.ToolTipRole)

            parent.appendRow([item, value])
            self.state_map[name] = item.index()
                    

    def update_state_variables(self):
        if self and self.init:
            for v in Callbacks.info.state:
                path = self.split_path(v.path)
                if len(path) == 0:
                    continue

                # add parents
                for (p_of_p, p) in path[:-1]:
                    if not p_of_p:
                        self.insert_parent(self.state_model.invisibleRootItem(), p)
                    else:
                        p_of_p_item = self.state_model.itemFromIndex(self.state_map[p_of_p])
                        self.insert_parent(p_of_p_item,p)
                    

                # add variable
                (parent, name) = path[-1]
                if name not in self.state_map:
                    self.insert_variable(self.get_parent(parent), name, v.value_as_json)
                else:
                    index = self.state_map[name]
                    value_index = index.siblingAtColumn(1)

                    value = self.state_model.itemFromIndex(value_index)
                    text = v.value_as_json #+ "_" + str(self.count)

                    if value.data(Qt.DisplayRole) != text:
                        value.setData(text, Qt.DisplayRole)
                        value.setData(text, Qt.ToolTipRole)
                        
            self.ability_plan.setText(str(Callbacks.info.ability_plan))
            self.enabled_ability_transitions.setText(str(Callbacks.info.enabled_ability_transitions))
            self.operation_plan.setText(str(Callbacks.info.operation_plan))
            self.enabled_operation_transitions.setText(str(Callbacks.info.enabled_operation_transitions))

            #self.count += 1

            #i = self.state_model.invisibleRootItem().index()
            #self.tree.dataChanged(i, i)
        

    def trigger(self):
        self.triggerSignal.emit()

    def info_widget(self):
        info_l = QGridLayout(self)

        self.ability_plan = QLabel("no info yet")
        info_l.addWidget(QLabel("ability_plan: "), 1, 0)
        info_l.addWidget(self.ability_plan, 1, 1)
        
        self.enabled_ability_transitions = QLabel("no info yet")
        info_l.addWidget(QLabel("enabled_ability_transitions: "), 2, 0)
        info_l.addWidget(self.enabled_ability_transitions, 2, 1)

        self.operation_plan = QLabel("no info yet")
        info_l.addWidget(QLabel("operation_plan: "), 3, 0)
        info_l.addWidget(self.operation_plan, 3, 1)

        self.enabled_operation_transitions = QLabel("no info yet")
        info_l.addWidget(QLabel("enabled_operation_transitions: "), 4, 0)
        info_l.addWidget(self.enabled_operation_transitions, 4, 1)

        box = QGroupBox("Runner info")
        box.setLayout(info_l)
        return box

    def tree_widget(self):
        self.state_model.setHorizontalHeaderLabels(['path', 'value'])
        self.state_model_proxy.setRecursiveFilteringEnabled(True)
        self.state_model_proxy.setFilterRole(Qt.ToolTipRole)
        self.state_model_proxy.setSourceModel(self.state_model)

        tree_l = QGridLayout(self)
        filter = QLineEdit("")
        def filter_changed(text):
            regex = "^.*{}.*".format(text)
            self.state_model_proxy.setFilterRegExp(QRegExp(regex, Qt.CaseInsensitive))
            self.tree.expandAll()

        filter.textChanged.connect(filter_changed)
        tree_l.addWidget(filter, 1, 0)

        expand_button = QPushButton("expand")
        def expand_button_clicked():
            print("expand")
            self.tree.expandAll()
        expand_button.clicked.connect(expand_button_clicked)
        tree_l.addWidget(expand_button, 1, 1)
        collaps_button = QPushButton("collaps")
        def collaps_button_clicked():
            print("collapse")
            self.tree.collapseAll()
        collaps_button.clicked.connect(collaps_button_clicked)
        tree_l.addWidget(collaps_button, 1, 2)

        self.tree = QTreeView(self)
        
        self.tree.header().setDefaultSectionSize(300)
        self.tree.setModel(self.state_model_proxy)
        self.tree.setSortingEnabled(True)
        tree_l.addWidget(self.tree, 2, 0, 1, 3)

        box = QGroupBox("State")
        box.setLayout(tree_l)
        return box


def main(args=None):

    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            node = Ros2Node()
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
