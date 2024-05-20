import rclpy
from .modules.sim_lattice_record_node import SimLatticeRecordNode



def main(args=None) -> None:
    rclpy.init()
    node = SimLatticeRecordNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()