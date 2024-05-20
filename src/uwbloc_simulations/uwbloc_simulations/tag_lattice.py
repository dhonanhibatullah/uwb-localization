import rclpy
import time
from .modules.tag_lattice_node import TagLatticeNode



def main(args=None) -> None:
    rclpy.init()
    node = TagLatticeNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()