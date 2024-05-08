import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import numpy as np



class TagNode(Node):

    def __init__(self) -> None:
        super().__init__('TagNode')

        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('broadcast_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('initial_position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('moving_omega', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('moving_radius', rclpy.Parameter.Type.DOUBLE)

        self.ID                 = self.get_parameter('id').value
        self.BROADCAST_PERIOD   = self.get_parameter('broadcast_period').value
        self.INITIAL_POSITION   = self.get_parameter('initial_position').value
        self.MOVING_OMEGA       = self.get_parameter('moving_omega').value
        self.MOVING_RADIUS      = self.get_parameter('moving_radius').value

        self.timer      = 0.0
        self.sn         = 0
        self.position   = self.INITIAL_POSITION

        self.broadcast_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.TagBroadcastTX,
            topic       = 'medium/sub/tag_broadcast',
            qos_profile = 10
        )

        self.broadcast_timer = self.create_timer(
            timer_period_sec    = self.BROADCAST_PERIOD,
            callback            = self.broadcastTimerCallback
        )



    def updatePosition(self) -> None:
        self.timer      += self.BROADCAST_PERIOD
        self.sn         = (self.sn + 1)%256
        self.position   = [
            self.INITIAL_POSITION[0] + self.MOVING_RADIUS*(1.0 - np.cos(self.MOVING_OMEGA*self.timer)),
            self.INITIAL_POSITION[1] + self.MOVING_RADIUS*np.sin(self.MOVING_OMEGA*self.timer),
            self.INITIAL_POSITION[2]
        ]



    def broadcastTimerCallback(self) -> None:
        broadcast_msg       = uwbloc_interfaces.TagBroadcastTX()
        broadcast_msg.id    = self.ID
        broadcast_msg.sn    = self.sn
        broadcast_msg.pos   = self.position

        self.broadcast_pub.publish(broadcast_msg)
        self.updatePosition()