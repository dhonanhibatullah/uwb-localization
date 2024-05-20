import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import numpy as np
import json



class MediumNode(Node):

    def __init__(self) -> None:
        super().__init__('MediumNode')

        self.declare_parameter('scaled_speed_of_light', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('scale_ratio', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('network_info', rclpy.Parameter.Type.STRING)

        self.SCALED_C       = self.get_parameter('scaled_speed_of_light').value
        self.SCALE_RATIO    = self.get_parameter('scale_ratio').value
        self.NETWORK_INFO   = json.loads(self.get_parameter('network_info').value)
        self.TAG_LIST       = self.NETWORK_INFO['tag_list']
        self.ANCHOR_LIST    = self.NETWORK_INFO['anchor_list']
        self.MASTER_CLOCK   = self.NETWORK_INFO['master_clock']

        self.tag_broadcast_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.TagBroadcastTX,
            topic       = 'medium/sub/tag_broadcast',
            callback    = self.tagBroadcastSubCallback,
            qos_profile = 1000
        )

        self.calib_poll_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.CalibrationPollTX,
            topic       = 'medium/sub/calib_poll',
            callback    = self.calibPollSubCallback,
            qos_profile = 1000
        )

        self.calib_resp_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.CalibrationRespTX,
            topic       = 'medium/sub/calib_resp',
            callback    = self.calibRespSubCallback,
            qos_profile = 1000
        )

        self.master_sync_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.MasterClockSyncTX,
            topic       = 'medium/sub/master_sync',
            callback    = self.masterSyncSubCallback,
            qos_profile = 1000
        )

        self.tag_broadcast_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.TagBroadcastRX,
            topic       = 'medium/pub/tag_broadcast',
            qos_profile = 1000
        )

        self.calib_poll_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.CalibrationPollRX,
            topic       = 'medium/pub/calib_poll',
            qos_profile = 1000
        )

        self.calib_resp_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.CalibrationRespRX,
            topic       = 'medium/pub/calib_resp',
            qos_profile = 1000
        )

        self.master_sync_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.MasterClockSyncRX,
            topic       = 'medium/pub/master_sync',
            qos_profile = 1000
        )



    def calculateToF2Anchors(self, pos:list, noise:bool=True) -> list:
        tof = []

        for anchor in self.ANCHOR_LIST:
            tof.append(
                (np.sqrt(
                    (pos[0] - anchor[1][0])**2.0 + 
                    (pos[1] - anchor[1][1])**2.0 +
                    (pos[2] - anchor[1][2])**2.0
                ) / self.SCALED_C + (3.3e-7*np.random.rand() if noise else 0.0))*1e+6
            )

        return tof 
    


    def calculateToF2MasterClock(self, pos:list, noise:bool=True) -> float:
        tof = (np.sqrt(
            (pos[0] - self.MASTER_CLOCK[0])**2.0 + 
            (pos[1] - self.MASTER_CLOCK[1])**2.0 +
            (pos[2] - self.MASTER_CLOCK[2])**2.0
        ) / self.SCALED_C + (6.6e-8*np.random.rand() if noise else 0.0))*1e+6
        return tof



    def tagBroadcastSubCallback(self, msg:uwbloc_interfaces.TagBroadcastTX) -> None:
        pub_msg     = uwbloc_interfaces.TagBroadcastRX()
        pub_msg.id  = msg.id
        pub_msg.sn  = msg.sn
        pub_msg.tof = self.calculateToF2Anchors(msg.pos, noise=True)

        self.tag_broadcast_pub.publish(pub_msg)



    def calibPollSubCallback(self, msg:uwbloc_interfaces.CalibrationPollTX) -> None:
        pub_msg     = uwbloc_interfaces.CalibrationPollRX()
        pub_msg.id  = msg.id
        pub_msg.tof = self.calculateToF2MasterClock(msg.pos, noise=False)

        self.calib_poll_pub.publish(pub_msg)



    def calibRespSubCallback(self, msg:uwbloc_interfaces.CalibrationRespTX) -> None:
        pub_msg             = uwbloc_interfaces.CalibrationRespRX()
        pub_msg.id          = msg.id
        pub_msg.poll_tof    = msg.poll_tof
        pub_msg.resp_tof    = msg.poll_tof

        self.calib_resp_pub.publish(pub_msg)



    def masterSyncSubCallback(self, msg:uwbloc_interfaces.MasterClockSyncTX) -> None:
        pub_msg         = uwbloc_interfaces.MasterClockSyncRX()
        pub_msg.sync_ts = msg.sync_ts
        pub_msg.tof     = self.calculateToF2Anchors(msg.pos, noise=False)

        self.master_sync_pub.publish(pub_msg)