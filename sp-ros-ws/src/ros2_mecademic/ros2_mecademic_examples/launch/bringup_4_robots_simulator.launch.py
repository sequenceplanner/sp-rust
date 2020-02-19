import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
     package_prefix = get_package_share_directory('ros2_mecademic_examples')

     namespace_1 = "robot1"
     namespace_2 = "robot2"
     namespace_3 = "robot3"
     namespace_4 = "robot4"

     mecademic_joint_states_1 = 'mecademic_joint_states'
     mecademic_esd_to_gui_1 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_1 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_1 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_1 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_1 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_1 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_1 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_1 = 'mecademic_utils_to_gui'

     mecademic_joint_states_2 = 'mecademic_joint_states'
     mecademic_esd_to_gui_2 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_2 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_2 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_2 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_2 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_2 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_2 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_2 = 'mecademic_utils_to_gui'

     mecademic_joint_states_3 = 'mecademic_joint_states'
     mecademic_esd_to_gui_3 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_3 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_3 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_3 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_3 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_3 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_3 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_3 = 'mecademic_utils_to_gui'

     mecademic_joint_states_4 = 'mecademic_joint_states'
     mecademic_esd_to_gui_4 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_4 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_4 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_4 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_4 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_4 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_4 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_4 = 'mecademic_utils_to_gui'

     rviz_config_file = os.path.join(get_package_share_directory('ros2_mecademic_examples'),
                        'config', 'meca_500_r3_4_robots.rviz')

     return LaunchDescription([

          DeclareLaunchArgument('namespace', default_value = namespace_1, description = 'namespace'),
          DeclareLaunchArgument('mecademic_joint_states', default_value = mecademic_joint_states_1, description = 'mecademic_joint_states'),
          DeclareLaunchArgument('mecademic_esd_to_gui', default_value = mecademic_esd_to_gui_1, description = 'mecademic_esd_to_gui'),
          DeclareLaunchArgument('mecademic_gui_to_esd', default_value = mecademic_gui_to_esd_1, description = 'mecademic_gui_to_esd'),
          DeclareLaunchArgument('mecademic_esd_to_sp', default_value = mecademic_esd_to_sp_1, description = 'mecademic_esd_to_sp'),
          DeclareLaunchArgument('mecademic_sp_to_esd', default_value = mecademic_sp_to_esd_1, description = 'mecademic_sp_to_esd'),
          DeclareLaunchArgument('mecademic_sp_to_utils', default_value = mecademic_sp_to_utils_1, description = 'mecademic_sp_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_sp', default_value = mecademic_utils_to_sp_1, description = 'mecademic_utils_to_sp'),
          DeclareLaunchArgument('mecademic_gui_to_utils', default_value = mecademic_gui_to_utils_1, description = 'mecademic_gui_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_gui', default_value = mecademic_utils_to_gui_1, description = 'mecademic_utils_to_gui'),

          DeclareLaunchArgument('namespace', default_value = namespace_2, description = 'namespace'),
          DeclareLaunchArgument('mecademic_joint_states', default_value = mecademic_joint_states_2, description = 'mecademic_joint_states'),
          DeclareLaunchArgument('mecademic_esd_to_gui', default_value = mecademic_esd_to_gui_2, description = 'mecademic_esd_to_gui'),
          DeclareLaunchArgument('mecademic_gui_to_esd', default_value = mecademic_gui_to_esd_2, description = 'mecademic_gui_to_esd'),
          DeclareLaunchArgument('mecademic_esd_to_sp', default_value = mecademic_esd_to_sp_2, description = 'mecademic_esd_to_sp'),
          DeclareLaunchArgument('mecademic_sp_to_esd', default_value = mecademic_sp_to_esd_2, description = 'mecademic_sp_to_esd'),
          DeclareLaunchArgument('mecademic_sp_to_utils', default_value = mecademic_sp_to_utils_2, description = 'mecademic_sp_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_sp', default_value = mecademic_utils_to_sp_2, description = 'mecademic_utils_to_sp'),
          DeclareLaunchArgument('mecademic_gui_to_utils', default_value = mecademic_gui_to_utils_2, description = 'mecademic_gui_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_gui', default_value = mecademic_utils_to_gui_2, description = 'mecademic_utils_to_gui'),

          DeclareLaunchArgument('namespace', default_value = namespace_3, description = 'namespace'),
          DeclareLaunchArgument('mecademic_joint_states', default_value = mecademic_joint_states_3, description = 'mecademic_joint_states'),
          DeclareLaunchArgument('mecademic_esd_to_gui', default_value = mecademic_esd_to_gui_3, description = 'mecademic_esd_to_gui'),
          DeclareLaunchArgument('mecademic_gui_to_esd', default_value = mecademic_gui_to_esd_3, description = 'mecademic_gui_to_esd'),
          DeclareLaunchArgument('mecademic_esd_to_sp', default_value = mecademic_esd_to_sp_3, description = 'mecademic_esd_to_sp'),
          DeclareLaunchArgument('mecademic_sp_to_esd', default_value = mecademic_sp_to_esd_3, description = 'mecademic_sp_to_esd'),
          DeclareLaunchArgument('mecademic_sp_to_utils', default_value = mecademic_sp_to_utils_3, description = 'mecademic_sp_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_sp', default_value = mecademic_utils_to_sp_3, description = 'mecademic_utils_to_sp'),
          DeclareLaunchArgument('mecademic_gui_to_utils', default_value = mecademic_gui_to_utils_3, description = 'mecademic_gui_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_gui', default_value = mecademic_utils_to_gui_3, description = 'mecademic_utils_to_gui'),

          DeclareLaunchArgument('namespace', default_value = namespace_4, description = 'namespace'),
          DeclareLaunchArgument('mecademic_joint_states', default_value = mecademic_joint_states_4, description = 'mecademic_joint_states'),
          DeclareLaunchArgument('mecademic_esd_to_gui', default_value = mecademic_esd_to_gui_4, description = 'mecademic_esd_to_gui'),
          DeclareLaunchArgument('mecademic_gui_to_esd', default_value = mecademic_gui_to_esd_4, description = 'mecademic_gui_to_esd'),
          DeclareLaunchArgument('mecademic_esd_to_sp', default_value = mecademic_esd_to_sp_4, description = 'mecademic_esd_to_sp'),
          DeclareLaunchArgument('mecademic_sp_to_esd', default_value = mecademic_sp_to_esd_4, description = 'mecademic_sp_to_esd'),
          DeclareLaunchArgument('mecademic_sp_to_utils', default_value = mecademic_sp_to_utils_4, description = 'mecademic_sp_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_sp', default_value = mecademic_utils_to_sp_4, description = 'mecademic_utils_to_sp'),
          DeclareLaunchArgument('mecademic_gui_to_utils', default_value = mecademic_gui_to_utils_4, description = 'mecademic_gui_to_utils'),
          DeclareLaunchArgument('mecademic_utils_to_gui', default_value = mecademic_utils_to_gui_4, description = 'mecademic_utils_to_gui'),

          # robot1
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/robot_1_bringup_simulator.launch.py']),
               launch_arguments = {'namespace': namespace_1,
                                   'mecademic_joint_states': mecademic_joint_states_1,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_1, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_1,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_1,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_1,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_1,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_1,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_1,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_1}.items()
          ),

           # robot2
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/robot_2_bringup_simulator.launch.py']),
               launch_arguments = {'namespace': namespace_2,
                                   'mecademic_joint_states': mecademic_joint_states_2,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_2, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_2,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_2,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_2,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_2,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_2,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_2,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_2}.items()
          ),

           # robot3
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/robot_3_bringup_simulator.launch.py']),
               launch_arguments = {'namespace': namespace_3,
                                   'mecademic_joint_states': mecademic_joint_states_3,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_3, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_3,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_3,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_3,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_3,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_3,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_3,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_3}.items()
          ),

           # robot4
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/robot_4_bringup_simulator.launch.py']),
               launch_arguments = {'namespace': namespace_4,
                                   'mecademic_joint_states': mecademic_joint_states_4,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_4, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_4,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_4,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_4,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_4,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_4,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_4,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_4}.items()
          ),

          Node(package='rviz2', node_executable='rviz2', output='screen', arguments=['-d', rviz_config_file]),
     ])
