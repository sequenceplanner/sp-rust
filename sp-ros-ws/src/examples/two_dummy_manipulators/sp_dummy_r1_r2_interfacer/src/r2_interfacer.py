import sys
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class R2Interfacer(Node):

    def __init__(self):
        super().__init__("r2_interfacer")

        self.from_sp = String()
        self.to_sp = String()
        self.from_r = JointState()
        self.to_r = JointState()
        self.tmr_period = 0.02
        self.tmr_period2 = 0.5

        self.act_pos_j1 = 0.0
        self.ref_pos_j1 = 0.0
        self.act_pos_j2 = 0.0
        self.ref_pos_j2 = 0.0
        self.act_pos_str = ""
        self.ref_pos_str = ""

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.joint_subscriber = self.create_subscription(
            JointState, 
            "/r2_joint_states",
            self.joint_callback,
            10)

        # Could be good to start the subscribers first so that they update the variables if other nodes are up
        self.cmd_subscriber = self.create_subscription(
            String, 
            "/r2_sp_to_interfacer",
            self.sp_callback,
            10)

        # Then sleep for a bit so that the node get the updated variables before publishing them
        time.sleep(2)

        self.state_to_sp_publisher_ = self.create_publisher(
            String,
            "/r2_interfacer_to_sp",
            10)
        
        self.joint_cmd_publisher_ = self.create_publisher(
            JointState,
            "/r2_joint_states",
            10)
    
        # Decouple message receiving and forwarding
        self.interfacer_to_sp_tmr = self.create_timer(
            self.tmr_period2, 
            self.interfacer_to_sp_publisher_callback)

        self.interfacer_to_r_tmr = self.create_timer(
            self.tmr_period, 
            self.interfacer_to_r_publisher_callback)

    def joint_callback(self, data):
        self.act_pos_j1 = data.position[0]
        self.act_pos_j2 = data.position[1]
        # print(self.act_pos)
        if self.act_pos_j1 == 0.0 and self.act_pos_j2 == 0.0:
            self.act_pos_str = "away"
        elif self.act_pos_j1 == -1.0 and self.act_pos_j2 == -1.0:
            self.act_pos_str = "at"
        else:
            pass


    def sp_callback(self, data):
        self.ref_pos_str = data.data
        if self.ref_pos_str == "at":
            self.ref_pos_j1 = -1.0
            self.ref_pos_j2 = -1.0
        elif self.ref_pos_str == "away":
            self.ref_pos_j1 = 0.0
            self.ref_pos_j2 = 0.0
        else:
            pass


    def interfacer_to_r_publisher_callback(self):
        if self.ref_pos_j1 < self.act_pos_j1:
            if self.ref_pos_j2 < self.act_pos_j2:
                self.to_r.name = ["r2_j1", "r2_j2"]
                self.to_r.position = [round(self.act_pos_j1 - 0.01, 2), round(self.act_pos_j2 - 0.01, 2)]
                self.joint_cmd_publisher_.publish(self.to_r)
            elif self.ref_pos_j2 > self.act_pos_j2:
                self.to_r.name = ["r2_j1", "r2_j2"]
                self.to_r.position = [round(self.act_pos_j1 - 0.01, 2), round(self.act_pos_j2 + 0.01, 2)]
                self.joint_cmd_publisher_.publish(self.to_r)
            else:
                pass
        elif self.ref_pos_j1 > self.act_pos_j1:
            if self.ref_pos_j2 < self.act_pos_j2:
                self.to_r.name = ["r2_j1", "r2_j2"]
                self.to_r.position = [round(self.act_pos_j1 + 0.01, 2), round(self.act_pos_j2 - 0.01, 2)]
                self.joint_cmd_publisher_.publish(self.to_r)
            elif self.ref_pos_j2 > self.act_pos_j2:
                self.to_r.name = ["r2_j1", "r2_j2"]
                self.to_r.position = [round(self.act_pos_j1 + 0.01, 2), round(self.act_pos_j2 + 0.01, 2)]
                self.joint_cmd_publisher_.publish(self.to_r)
            else:
                pass
        else:
            pass

        
    def interfacer_to_sp_publisher_callback(self):
        if self.ref_pos_str == "at" and self.act_pos_str == "at":
            self.to_sp.data = "at"
        elif self.ref_pos_str == "away" and self.act_pos_str == "away":
            self.to_sp.data = "away"
        else:
            self.to_sp.data = "unknown"

        self.state_to_sp_publisher_.publish(self.to_sp)

def main(args=None):
    rclpy.init(args=args)

    unicorn_1_interfacer = R2Interfacer()

    rclpy.spin(unicorn_1_interfacer)

    unicorn_1_interfacer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()