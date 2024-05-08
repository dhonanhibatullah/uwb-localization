import rclpy
from .modules.medium_node import MediumNode



def main(args=None) -> None:
    rclpy.init()
    node = MediumNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()