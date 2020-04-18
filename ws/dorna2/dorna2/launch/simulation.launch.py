import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


     dorna2_share = FindPackageShare('dorna2').find('dorna2')
     urdf = os.path.join(dorna2_share, 'urdf', 'dorna.urdf')
     with open(urdf, 'r') as infp:
        robot_desc = infp.read()

     robot_state_publisher_params = {
         "robot_description": robot_desc,
         "frame_id_prefix": "dorna/r1/"
     }    
     rsp_node = launch_ros.actions.Node(package='robot_state_publisher', 
                                        node_executable='robot_state_publisher',
                                        node_namespace='dorna/r1',
                                        output='screen',   
                                        parameters=[robot_state_publisher_params]
                                        )

     simulation_parameters = {
          "saved_poses_file": os.path.join(dorna2_share, 'launch', 'r1_joint_poses.csv'),
          "joint_names": ["dorna_axis_1_joint",
                    "dorna_axis_2_joint",
                    "dorna_axis_3_joint",
                    "dorna_axis_4_joint",
                    "dorna_axis_5_joint",
                    "nonexistent"]
     }
     sim_node = launch_ros.actions.Node(package='robot_simulator',
                     node_executable='robot_simulator',
                     node_namespace='dorna/r1',
                     output='screen',
                     parameters=[simulation_parameters]
                     )



     return launch.LaunchDescription([rsp_node, sim_node])
          
          # Node(package='ros2_dorna_gui', node_executable='ros2_dorna_gui',
          #      output='screen', condition = IfCondition(LaunchConfiguration("gui"))),
          # Node(package='ros2_dorna_utilities', node_executable='ros2_dorna_utilities',
          #      output='screen'),
          # Node(package='rviz2', node_executable='rviz2', arguments=['-d', rviz_config_file],
          #      output='screen', condition = IfCondition(LaunchConfiguration("rviz")))