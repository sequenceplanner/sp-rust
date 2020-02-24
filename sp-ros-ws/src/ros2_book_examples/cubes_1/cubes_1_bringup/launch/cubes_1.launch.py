import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
     package_prefix = get_package_share_directory('cubes_1_bringup')

     namespace_1 = "cubes/r1"
     namespace_2 = "cubes/r2"
     namespace_3 = "cubes/c1"

     mecademic_joint_states_1 = 'mecademic_joint_states'
     mecademic_esd_to_gui_1 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_1 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_1 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_1 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_1 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_1 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_1 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_1 = 'mecademic_utils_to_gui'
     state1 = 'state'
     command1 = 'command'

     mecademic_joint_states_2 = 'mecademic_joint_states'
     mecademic_esd_to_gui_2 = 'mecademic_esd_to_gui'
     mecademic_gui_to_esd_2 = 'mecademic_gui_to_esd'
     mecademic_esd_to_sp_2 = 'mecademic_esd_to_sp'
     mecademic_sp_to_esd_2 = 'mecademic_sp_to_esd'
     mecademic_sp_to_utils_2 = 'mecademic_sp_to_utils'
     mecademic_utils_to_sp_2 = 'mecademic_utils_to_sp'
     mecademic_gui_to_utils_2 = 'mecademic_gui_to_utils'
     mecademic_utils_to_gui_2 = 'mecademic_utils_to_gui'
     state2 = 'state'
     command2 = 'command'

     rviz_config_file = os.path.join(get_package_share_directory('cubes_1_bringup'),
                        'config', 'cubes_1.rviz')

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
          DeclareLaunchArgument('mecademic_utils_to_gui', default_value = mecademic_utils_to_gui_1, description = 'mecademic_utils_to_gui'),
          DeclareLaunchArgument('state', default_value = state1, description = 'state'),
          DeclareLaunchArgument('command', default_value = command1, description = 'command'),

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
          DeclareLaunchArgument('state', default_value = state2, description = 'state'),
          DeclareLaunchArgument('command', default_value = command2, description = 'command'),

          # robot1
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/cubes_1_meca_1.launch.py']),
               launch_arguments = {'namespace': namespace_1,
                                   'mecademic_joint_states': mecademic_joint_states_1,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_1, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_1,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_1,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_1,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_1,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_1,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_1,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_1,
                                   'command': command1,
                                   'state': state1}.items()
          ),

          # robot2
          IncludeLaunchDescription(
               PythonLaunchDescriptionSource([package_prefix, '/launch/cubes_1_meca_2.launch.py']),
               launch_arguments = {'namespace': namespace_2,
                                   'mecademic_joint_states': mecademic_joint_states_2,
                                   'mecademic_esd_to_gui': mecademic_esd_to_gui_2, 
                                   'mecademic_gui_to_esd': mecademic_gui_to_esd_2,
                                   'mecademic_esd_to_sp': mecademic_esd_to_sp_2,
                                   'mecademic_sp_to_esd': mecademic_sp_to_esd_2,
                                   'mecademic_sp_to_utils': mecademic_sp_to_utils_2,
                                   'mecademic_utils_to_sp': mecademic_utils_to_sp_2,
                                   'mecademic_gui_to_utils': mecademic_gui_to_utils_2,
                                   'mecademic_utils_to_gui': mecademic_utils_to_gui_2,
                                   'command': command2,
                                   'state': state2}.items()
          ),

          # #cube1
          # IncludeLaunchDescription(
          #     PythonLaunchDescriptionSource([package_prefix, '/launch/cubes_1_cube_1.launch.py']),
          #     launch_arguments = {'namespace': namespace_3}.items()
          # ),

          Node(package='rviz2', node_executable='rviz2', output='screen', arguments=['-d', rviz_config_file]),
          Node(package='ros2_scene_manipulation', node_executable='service_main', output='screen'),
          Node(package='cubes_1_scene_master', node_executable='cubes_1_scene_master', output='screen'),
     ])