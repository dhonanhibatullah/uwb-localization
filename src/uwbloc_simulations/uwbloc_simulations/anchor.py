import rclpy
from .modules.anchor_node import AnchorNode



def main(args=None) -> None:
    rclpy.init()
    node = AnchorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()