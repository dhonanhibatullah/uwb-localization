import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import time
import json



class AnchorNode(Node):

    def __init__(self) -> None:
        super().__init__('AnchorNode')

        self.declare_parameter('scaled_speed_of_light', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('scale_ratio', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('position', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('time_drift', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('calib_period', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('calib_cnt', rclpy.Parameter.Type.INTEGER)

        self.SCALED_C       = self.get_parameter('scaled_speed_of_light').value
        self.SCALE_RATIO    = self.get_parameter('scale_ratio').value
        self.ID             = self.get_parameter('id').value
        self.POSITION       = self.get_parameter('position').value
        self.TIME_DRIFT     = self.get_parameter('time_drift').value
        self.CALIB_PERIOD   = self.get_parameter('calib_period').value
        self.CALIB_CNT      = self.get_parameter('calib_cnt').value
        self.INDEX          = int(self.ID.split('_')[-1])

        self.synchronized       = False
        self.sync_time          = []
        self.machine_time       = []

        self.calibrated         = False
        self.calibration_data   = []
        self.calibration_cnt    = 0
        self.master_clock_tof   = 0.0

        self.data_exist = False
        self.tag_set    = set(())
        self.tag_data   = dict(())
        self.server_msg = dict(())

        self.tag_broadcast_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.TagBroadcastRX,
            topic       = 'medium/pub/tag_broadcast',
            callback    = self.tagBroadcastSubCallback,
            qos_profile = 10
        )

        self.calib_resp_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.CalibrationRespRX,
            topic       = 'medium/pub/calib_resp',
            callback    = self.calibRespSubCallback,
            qos_profile = 10
        )

        self.master_sync_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.MasterClockSyncRX,
            topic       = 'medium/pub/master_sync',
            callback    = self.masterSyncSubCallback,
            qos_profile = 10
        )

        self.server_call_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.ServerCall,
            topic       = 'server/call',
            callback    = self.serverCallSubCallback,
            qos_profile = 1000
        )

        self.calib_poll_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.CalibrationPollTX,
            topic       = 'medium/sub/calib_poll',
            qos_profile = 10
        )

        self.server_reply_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.AnchorInfo,
            topic       = 'server/reply',
            qos_profile = 1000
        )

        self.calibration_timer = self.create_timer(
            timer_period_sec    = self.CALIB_PERIOD,
            callback            = self.calibrationTimerCallback
        )


    
    def tagBroadcastSubCallback(self, msg:uwbloc_interfaces.TagBroadcastRX) -> None:
        rx_time = time.time_ns()*1e-9 + msg.tof[self.INDEX]

        if self.calibrated and self.synchronized:
            if not (msg.id in self.tag_set):
                self.tag_set.add(msg.id)
                self.tag_data.update({msg.id: [[],[]]})

            self.tag_data[msg.id][0].append(msg.sn)
            self.tag_data[msg.id][1].append(rx_time)



    def calibRespSubCallback(self, msg:uwbloc_interfaces.CalibrationRespRX) -> None:
        if (not self.calibrated) and (msg.id == self.ID):
            self.calibration_data.append(
                (msg.poll_tof + msg.resp_tof)/2.0
            )

            self.calibration_cnt += 1

            if self.calibration_cnt == self.CALIB_CNT:

                for data in self.calibration_data:
                    self.master_clock_tof += data

                self.master_clock_tof /= float(self.CALIB_CNT)

                self.calibrated = True
                self.calibration_timer.destroy()

                self.get_logger().info(f'{self.ID} calibrated! [ToF: {self.master_clock_tof}]')



    def masterSyncSubCallback(self, msg:uwbloc_interfaces.MasterClockSyncRX) -> None:
        rx_time = time.time_ns()*1e-9 + msg.tof[self.INDEX]

        if self.calibrated:
            if not self.synchronized:
                if len(self.sync_time) == 1:
                    self.synchronized = True

                self.sync_time.append(msg.sync_ts + self.master_clock_tof)
                self.machine_time.append(rx_time)

            else:
                self.data_exist = True
                
                self.server_msg.clear()
                self.server_msg.update({'sync_time': self.sync_time.copy()})
                self.server_msg.update({'machine_time': self.machine_time.copy()})
                self.server_msg.update({'tag_data': self.tag_data.copy()})

                self.sync_time.pop(0)
                self.machine_time.pop(0)
                self.tag_data.clear()
                self.tag_set.clear()

                self.sync_time.append(msg.sync_ts + self.master_clock_tof)
                self.machine_time.append(rx_time)



    def serverCallSubCallback(self, msg:uwbloc_interfaces.ServerCall) -> None:
        if self.data_exist:
            pub_msg         = uwbloc_interfaces.AnchorInfo()
            pub_msg.id      = self.ID
            pub_msg.info    = json.dumps(self.server_msg.copy())

            self.server_reply_pub.publish(pub_msg)



    def calibrationTimerCallback(self) -> None:
        if not self.calibrated:
            poll_msg        = uwbloc_interfaces.CalibrationPollTX()
            poll_msg.id     = self.ID
            poll_msg.pos    = self.POSITION

            self.calib_poll_pub.publish(poll_msg)

