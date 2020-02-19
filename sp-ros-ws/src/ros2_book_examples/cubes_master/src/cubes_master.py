import sys
import rclpy

from rclpy.node import Node
from cubes_msgs.msg import SMCommand
from cubes_msgs.msg import SMState

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

class CubesMaster(Node):

    def __init__(self):
        super().__init__("cubes_master")

        self.sm_state_timer_period = 0.2

        self.sm_command = SMCommand()
        self.sm_command.attach_r1_box = False                       
        self.sm_command.attach_r2_box = False                 

        self.sm_command_subscriber = self.create_subscription(
            SMCommand, 
            "cubes/sm/command",
            self.sp_command_callback,
            10)
        
        time.sleep(2)

        self.sm_state = SMState()
        self.sm_state.attached_r1_box = False
        self.sm_state.attached_r2_box = False

        self.sm_state_publisher_ = self.create_publisher(
            SMState,
            "cubes/sm/state",
            10)

        self.sm_state_timer = self.create_timer(
            self.sm_state_timer_period, 
            self.sm_state_publisher_callback)

    def sp_command_callback(self, data):
        self.sm_command.attach_r1_box = data.attach_r1_box
        self.sm_command.attach_r2_box = data.attach_r2_box
        if self.sm_command.attach_r1_box == True and self.sm_command.attach_r2_box == False:
            self.fn_attach_r1_box()
        elif self.sm_command.attach_r2_box == True and self.sm_command.attach_r1_box == False:
            self.fn_attach_r2_box()
        else:
            pass

    def fn_attach_r1_box(self):
        ...
        return LaunchDescription([
            Node(package='cubes_sm', node_executable='service_main', output='screen')
        ])

    def fn_attach_r1_box(self):
        return LaunchDescription([
            Node(package='cubes_sm', node_executable='service_main', output='screen')
        ])

def main(args=None):
    rclpy.init(args=args)
    cubes_master = CubesMaster()
    rclpy.spin(cubes_master)
    cubes_master.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()