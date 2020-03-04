import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchContext

def generate_launch_description():
     # We need this otherwise python will never display our logging info...
     lc = LaunchContext()
     SetEnvironmentVariable('PYTHONUNBUFFERED', '1').visit(lc)

     urdf1 = os.path.join(get_package_share_directory('ros2_dorna_description'),
                          'urdf', 'dorna.urdf')
     rviz_config_file = os.path.join(get_package_share_directory('ros2_dorna_bringup'),
                                     'config', 'dorna.rviz')

     return LaunchDescription([

          DeclareLaunchArgument(
               name="gui",
               default_value="True",
               description="Launch GUI?"),

          DeclareLaunchArgument(
               name="rviz",
               default_value="True",
               description="Launch RViz?"),

          DeclareLaunchArgument(
               name="homing",
               default_value="False",
               description="Perform Dorna Homing procedure"),

          Node(package='ros2_dorna_state_publisher', node_executable='ros2_dorna_state_publisher',
               output='screen', arguments=[urdf1]),
          Node(package='ros2_dorna_driver', node_executable='ros2_dorna_driver',
               output='screen', arguments=[LaunchConfiguration("homing")]),
          Node(package='ros2_dorna_gui', node_executable='ros2_dorna_gui',
               output='screen', condition = IfCondition(LaunchConfiguration("gui"))),
          Node(package='ros2_dorna_utilities', node_executable='ros2_dorna_utilities',
               output='screen'),
          Node(package='rviz2', node_executable='rviz2', arguments=['-d', rviz_config_file],
               output='screen', condition = IfCondition(LaunchConfiguration("rviz")))
     ])
