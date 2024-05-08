import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import time



class MasterClockNode(Node):

    def __init__(self) -> None:
        super().__init__('MasterClockNode')

        self.declare_parameter('position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('sync_period', rclpy.Parameter.Type.DOUBLE)

        self.POSITION       = self.get_parameter('position').value
        self.SYNC_PERIOD    = self.get_parameter('sync_period').value

        self.calib_poll_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.CalibrationPollRX,
            topic       = 'medium/pub/calib_poll',
            callback    = self.calibPollSubCallback,
            qos_profile = 10
        )

        self.calib_resp_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.CalibrationRespTX,
            topic       = 'medium/sub/calib_resp',
            qos_profile = 10
        )

        self.master_sync_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.MasterClockSyncTX,
            topic       = 'medium/sub/master_sync',
            qos_profile = 10
        )

        self.sync_timer = self.create_timer(
            timer_period_sec    = self.SYNC_PERIOD,
            callback            = self.syncTimerCallback
        )



    def calibPollSubCallback(self, msg:uwbloc_interfaces.CalibrationPollRX) -> None:
        pub_msg             = uwbloc_interfaces.CalibrationRespTX()
        pub_msg.id          = msg.id
        pub_msg.poll_tof    = msg.tof

        self.calib_resp_pub.publish(pub_msg)



    def syncTimerCallback(self) -> None:
        tx_time = time.time_ns()*1e-9

        pub_msg         = uwbloc_interfaces.MasterClockSyncTX()
        pub_msg.sync_ts = tx_time
        pub_msg.pos     = self.POSITION

        self.master_sync_pub.publish(pub_msg)