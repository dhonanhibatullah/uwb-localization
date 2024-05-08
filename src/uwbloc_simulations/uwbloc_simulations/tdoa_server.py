import rclpy
from .modules.tdoa_server_node import TDOAServerNode



def main(args=None) -> None:
    rclpy.init()
    node = TDOAServerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()