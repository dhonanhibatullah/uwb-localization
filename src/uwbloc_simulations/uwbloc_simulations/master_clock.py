import rclpy
from .modules.master_clock_node import MasterClockNode



def main(args=None) -> None:
    rclpy.init()
    node = MasterClockNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()