import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import numpy as np
import json



class TDOAServerNode(Node):

    def __init__(self) -> None:
        super().__init__('AnchorNode')

        self.declare_parameter('scaled_speed_of_light', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('scale_ratio', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('network_info', rclpy.Parameter.Type.STRING)
        self.declare_parameter('server_call_period', rclpy.Parameter.Type.DOUBLE)

        self.SCALED_C       = self.get_parameter('scaled_speed_of_light').value
        self.SCALE_RATIO    = self.get_parameter('scale_ratio').value
        self.NETWORK_INFO   = json.loads(self.get_parameter('network_info').value)
        self.CALL_PERIOD    = self.get_parameter('server_call_period').value
        self.TAG_LIST       = self.NETWORK_INFO['tag_list']
        self.ANCHOR_LIST    = self.NETWORK_INFO['anchor_list']
        self.MASTER_CLOCK   = self.NETWORK_INFO['master_clock']

        self.anchor_set     = set(())
        self.anchor_data    = dict(())

        self.server_reply_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.AnchorInfo,
            topic       = 'server/reply',
            callback    = self.serverReplySubCallback,
            qos_profile = 1000
        )

        self.server_call_pub = self.create_publisher(
            msg_type    = uwbloc_interfaces.ServerCall,
            topic       = 'server/call',
            qos_profile = 1000
        )

        self.server_timer = self.create_timer(
            timer_period_sec    = self.CALL_PERIOD,
            callback            = self.serverTimerCallback
        )



    def isAnchorDataCompleted(self) -> bool:
        for anchor in self.ANCHOR_LIST:
            if not (anchor[0] in self.anchor_set):
                return False

        return True 



    def serverReplySubCallback(self, msg:uwbloc_interfaces.AnchorInfo) -> None:
        self.get_logger().info(f'Reply from {msg.id} received!')

        if not (msg.id in self.anchor_set):
            self.anchor_set.add(msg.id)
            self.anchor_data.update({msg.id: json.loads(msg.info)})
            
        if self.isAnchorDataCompleted():
            self.get_logger().info(f'ALL DATA RECEIVED!')

            for anchor in self.ANCHOR_LIST:
                pass


    
    def serverTimerCallback(self) -> None:
        pub_msg         = uwbloc_interfaces.ServerCall()
        pub_msg.data    = 101

        self.server_call_pub.publish(pub_msg)

        self.anchor_set.clear()
        self.anchor_data.clear()