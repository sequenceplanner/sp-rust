import sys
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SPDummyDoorInterfacer(Node):

    def __init__(self):
        super().__init__("sp_dummy_door_interfacer")

        self.from_sp = String()
        self.to_sp = String()
        self.from_door = JointState()
        self.to_door = JointState()
        self.tmr_period = 0.02
        self.tmr_period2 = 0.5

        self.act_pos = 0.0
        self.ref_pos = 0.0
        self.act_pos_str = ""
        self.ref_pos_str = ""

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.joint_subscriber = self.create_subscription(
            JointState, 
            "/door_joint_states",
            self.joint_callback,
            10)

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.cmd_subscriber = self.create_subscription(
            String, 
            "/dummy_door_sp_to_interfacer",
            self.sp_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        self.state_to_sp_publisher_ = self.create_publisher(
            String,
            "/dummy_door_interfacer_to_sp",
            10)
        
        self.joint_cmd_publisher_ = self.create_publisher(
            JointState,
            "/door_joint_states",
            10)
    
        # Decouple message receiving and forwarding
        self.interfacer_to_sp_tmr = self.create_timer(
            self.tmr_period2, 
            self.interfacer_to_sp_publisher_callback)

        self.interfacer_to_door_tmr = self.create_timer(
            self.tmr_period, 
            self.interfacer_to_door_publisher_callback)

    def joint_callback(self, data):
        self.act_pos = data.position[0]
        # print(self.act_pos)
        if self.act_pos == 0.0:
            self.act_pos_str = "closed"
        elif self.act_pos == 1.57:
            self.act_pos_str = "open"
        else:
            pass

    def sp_callback(self, data):
        self.ref_pos_str = data.data
        # print(data.data)
        if self.ref_pos_str == "open":
            self.ref_pos = 1.57
        elif self.ref_pos_str == "closed":
            self.ref_pos = 0.0
        else:
            pass

    # Publish init values of vars to the driver node or the latest receive cmd from SP
    def interfacer_to_door_publisher_callback(self):
        
        # if self.act_pos:
        if self.ref_pos < self.act_pos:
            # print(self.ref_pos, self.act_pos)
            # self.act_pos = self.act_pos + 0.01
            self.to_door.name = ["door_world"]
            self.to_door.position = [round(self.act_pos - 0.01, 2)]
            self.joint_cmd_publisher_.publish(self.to_door)
        elif self.ref_pos > self.act_pos:
            # print(self.ref_pos, self.act_pos)
            # self.act_pos = self.act_pos - 0.01
            self.to_door.name = ["door_world"]
            self.to_door.position = [round(self.act_pos + 0.01, 2)]
            self.joint_cmd_publisher_.publish(self.to_door)
        else:
            pass

        
    
    # Publish init values of vars to the driver node or the latest receive cmd from SP
    def interfacer_to_sp_publisher_callback(self):
        if self.ref_pos_str == "open" and self.act_pos_str == "closed":
            self.to_sp.data = "opening"
        elif self.ref_pos_str == "closed" and self.act_pos_str == "open":
            self.to_sp.data = "closing"
        elif self.ref_pos_str == "closed" and self.act_pos_str == "closed":
            self.to_sp.data = "closed"
        elif self.ref_pos_str == "open" and self.act_pos_str == "open":
            self.to_sp.data = "open"
        else:
            pass

        self.state_to_sp_publisher_.publish(self.to_sp)

def main(args=None):
    rclpy.init(args=args)

    unicorn_1_interfacer = SPDummyDoorInterfacer()

    rclpy.spin(unicorn_1_interfacer)

    unicorn_1_interfacer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
