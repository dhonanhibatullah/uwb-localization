import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import std_msgs.msg as std_msgs
import numpy as np



class TagLatticeNode(Node):

    def __init__(self) -> None:
        super().__init__('TagLatticeNode')

        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('broadcast_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('grid_x', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('grid_y', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('height', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('step', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('trial', rclpy.Parameter.Type.INTEGER)

        self.ID                 = self.get_parameter('id').value
        self.BROADCAST_PERIOD   = self.get_parameter('broadcast_period').value
        self.GRID_X             = self.get_parameter('grid_x').value
        self.GRID_Y             = self.get_parameter('grid_y').value
        self.HEIGHT             = self.get_parameter('height').value
        self.GRID_STEP          = self.get_parameter('step').value
        self.TRIAL              = self.get_parameter('trial').value

        self.grid_x_val = np.arange(self.GRID_X[0], self.GRID_X[1], self.GRID_STEP).tolist()
        self.grid_x_len = len(self.grid_x_val)
        self.grid_y_val = np.arange(self.GRID_Y[0], self.GRID_Y[1], self.GRID_STEP).tolist()
        self.grid_y_len = len(self.grid_y_val)

        self.sn         = 0
        self.position   = [self.grid_x_val[0], self.grid_y_val[0], self.HEIGHT]
        self.pos_cnt    = 0
        self.pos_x_cnt  = 0
        self.pos_y_cnt  = 0

        self.anchor_calibrating = True
        self.calib_cnt          = 0

        self.actual_pos_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.Position3D,
            topic       = f'{self.ID}/actual_position',
            qos_profile = 1000
        )

        self.broadcast_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.TagBroadcastTX,
            topic       = 'medium/sub/tag_broadcast',
            qos_profile = 10
        )

        self.terminator_pub = self.create_publisher(
            msg_type    = std_msgs.UInt8,
            topic       = f'{self.ID}/terminator',
            qos_profile = 1000
        )

        self.broadcast_timer = self.create_timer(
            timer_period_sec    = self.BROADCAST_PERIOD,
            callback            = self.broadcastTimerCallback
        )



    def updatePosition(self) -> None:
        if self.anchor_calibrating:
            pub_msg         = uwbloc_interfaces.Position3D()
            pub_msg.pos_3d  = self.position.copy()
            self.actual_pos_pub.publish(pub_msg)

            self.calib_cnt += 1

            if self.calib_cnt >= int(self.TRIAL*5):
                self.anchor_calibrating = False

        else:
            pub_msg         = uwbloc_interfaces.Position3D()
            pub_msg.pos_3d  = self.position.copy()
            self.actual_pos_pub.publish(pub_msg)

            self.sn         = (self.sn + 1)%256
            self.pos_cnt    = (self.pos_cnt + 1)%self.TRIAL

            if self.pos_cnt == 0:
                self.pos_x_cnt += 1

                if self.pos_x_cnt == self.grid_x_len:
                    self.pos_x_cnt = 0
                    self.pos_y_cnt += 1

                if (self.pos_x_cnt == 0) and (self.pos_y_cnt == self.grid_y_len):
                    self.get_logger().info('TAG GRID EXPERIMENT DONE')
                    self.pos_y_cnt = 0

                    terminator_msg      = std_msgs.UInt8()
                    terminator_msg.data = 69 
                    self.terminator_pub.publish(terminator_msg)
                    return

            self.position   = [
                self.grid_x_val[self.pos_x_cnt],
                self.grid_y_val[self.pos_y_cnt],
                self.HEIGHT
            ]



    def broadcastTimerCallback(self) -> None:
        broadcast_msg       = uwbloc_interfaces.TagBroadcastTX()
        broadcast_msg.id    = self.ID
        broadcast_msg.sn    = self.sn
        broadcast_msg.pos   = self.position

        self.broadcast_pub.publish(broadcast_msg)
        self.updatePosition()