import rclpy
from .modules.tag_node import TagNode



def main(args=None) -> None:
    rclpy.init()
    node = TagNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()