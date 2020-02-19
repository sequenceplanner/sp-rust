import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

     urdf = os.path.join(get_package_share_directory('cubes'),
                        'urdf', 'meca_2.urdf')

     namespace = LaunchConfiguration('namespace', default='')
     mecademic_joint_states = LaunchConfiguration('mecademic_joint_states', default='')
     mecademic_esd_to_gui = LaunchConfiguration('mecademic_esd_to_gui', default='')
     mecademic_gui_to_esd = LaunchConfiguration('mecademic_gui_to_esd', default='')
     mecademic_esd_to_sp = LaunchConfiguration('mecademic_esd_to_sp', default='')
     mecademic_sp_to_esd = LaunchConfiguration('mecademic_sp_to_esd', default='')
     mecademic_sp_to_utils = LaunchConfiguration('mecademic_sp_to_utils', default='')
     mecademic_utils_to_sp = LaunchConfiguration('mecademic_utils_to_sp', default='')
     mecademic_gui_to_utils = LaunchConfiguration('mecademic_gui_to_utils', default='')
     mecademic_utils_to_gui = LaunchConfiguration('mecademic_utils_to_gui', default='')
     command2 = LaunchConfiguration('command', default='')
     state2 = LaunchConfiguration('state', default='')
     
     return LaunchDescription([

          Node(package = 'ros2_mecademic_state_publisher', 
               node_executable = 'ros2_mecademic_state_publisher', 
               node_namespace = namespace,
               output = 'screen', 
               arguments=[urdf]),

          Node(package = 'ros2_mecademic_simulator', 
               node_executable = 'ros2_mecademic_simulator', 
               node_namespace = namespace,
               output='screen', 
               remappings=[('/mecademic_joint_states', mecademic_joint_states),
                           ('/mecademic_esd_to_gui', mecademic_esd_to_gui), 
                           ('/mecademic_gui_to_esd', mecademic_gui_to_esd),
                           ('/mecademic_sp_to_esd', mecademic_sp_to_esd),
                           ('/mecademic_esd_to_sp', mecademic_esd_to_sp),
                           ('/state', state2),
                           ('/command', command2)],
               arguments=[namespace]),
               

          Node(package='ros2_mecademic_gui',
               node_executable='ros2_mecademic_gui', 
               node_namespace=namespace,
               output='screen', 
               remappings=[('/mecademic_joint_states', mecademic_joint_states),
                           ('/mecademic_esd_to_gui', mecademic_esd_to_gui), 
                           ('/mecademic_gui_to_esd', mecademic_gui_to_esd),
                           ('/mecademic_utils_to_gui', mecademic_utils_to_gui),
                           ('/mecademic_gui_to_utils', mecademic_gui_to_utils)],
               arguments=[namespace]),

          Node(package='ros2_mecademic_utilities', 
               node_executable='ros2_mecademic_utilities', 
               node_namespace=namespace,
               output='screen',
               remappings=[('/mecademic_joint_states', mecademic_joint_states),
                           ('/mecademic_sp_to_utils', mecademic_sp_to_utils), 
                           ('/mecademic_utils_to_sp', mecademic_utils_to_sp),
                           ('/mecademic_utils_to_gui', mecademic_utils_to_gui),
                           ('/mecademic_gui_to_utils', mecademic_gui_to_utils)],
               arguments=[namespace]),
     ])
