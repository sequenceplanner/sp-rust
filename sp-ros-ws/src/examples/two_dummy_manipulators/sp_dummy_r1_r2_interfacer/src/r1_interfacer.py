import sys
import rclpy
from .interfacer import Interfacer

def main(args=None):
    rclpy.init(args=args)

    interfacer = Interfacer("r1")

    rclpy.spin(interfacer)

    interfacer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
