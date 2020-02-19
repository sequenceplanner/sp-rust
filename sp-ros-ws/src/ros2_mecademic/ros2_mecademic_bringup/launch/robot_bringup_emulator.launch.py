import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
     urdf1 = os.path.join(get_package_share_directory('ros2_mecademic_description'),
                        'urdf', 'meca_500_r3.urdf')

     namespace = LaunchConfiguration('node_namespace', default='')

     return LaunchDescription([
          Node(package='ros2_mecademic_state_publisher', node_executable='ros2_mecademic_state_publisher',
               output='screen', arguments=[urdf1, namespace]),
          Node(package='ros2_mecademic_emulator', node_executable='ros2_mecademic_emulator',
               output='screen', arguments=[namespace]),
          Node(package='ros2_mecademic_interfacer', node_executable='ros2_mecademic_interfacer',
               output='screen', arguments=[namespace]),
          Node(package='ros2_mecademic_gui', node_executable='ros2_mecademic_gui',
               output='screen', arguments=[namespace]),
          Node(package='ros2_mecademic_utilities', node_executable='ros2_mecademic_utilities',
               output='screen', arguments=[namespace]),
          # Node(package='rviz2', node_executable='rviz2',
          #      output='screen'),
     ])