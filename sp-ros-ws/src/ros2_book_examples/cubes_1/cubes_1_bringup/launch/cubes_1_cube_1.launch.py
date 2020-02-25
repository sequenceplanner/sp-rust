import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

     urdf = os.path.join(get_package_share_directory('cubes_1_bringup'),
                        'urdf', 'cube_1.urdf')

     namespace = LaunchConfiguration('namespace', default='')
     
     return LaunchDescription([

          Node(package = 'ros2_mecademic_state_publisher', 
               node_executable = 'ros2_mecademic_state_publisher', 
               node_namespace = namespace,
               output = 'screen', 
               arguments=[urdf])
     ])
