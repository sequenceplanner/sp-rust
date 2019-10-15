# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    urdf1 = os.path.join(get_package_share_directory('sp_dummy_robot_bringup'),
                        'launch', 'single_rrbot.urdf')
    urdf2 = os.path.join(get_package_share_directory('sp_dummy_robot_bringup'),
                         'launch', '2single_rrbot.urdf')
    return LaunchDescription([
        # Node(package='sp_dummy_map_server', node_executable='dummy_map_server', output='screen'),
        Node(package='sp_dummy_robot_state_publisher', node_executable='sp_dummy_robot_state_publisher',
             output='screen', arguments=[urdf1]),
        Node(package='sp_dummy_robot_state_publisher2', node_executable='sp_dummy_robot_state_publisher2',
             output='screen', arguments=[urdf2]),
     #     Node(package='sp_dummy_sensors', node_executable='dummy_joint_states', output='screen'),
     #    Node(package='sp_dummy_sensors2', node_executable='dummy_joint_states', output='screen'),
     #    Node(package='sp_dummy_sensors', node_executable='dummy_laser', output='screen')
    ])
