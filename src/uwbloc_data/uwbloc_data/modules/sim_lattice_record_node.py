import rclpy
from rclpy.node import Node
import uwbloc_interfaces.msg as uwbloc_interfaces
import std_msgs.msg as std_msgs
import yaml
import numpy as np



class SimLatticeRecordNode(Node):

    def __init__(self) -> None:
        super().__init__('SimLatticeRecordNode')

        self.declare_parameter('trial', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('filename', rclpy.Parameter.Type.STRING)

        self.TRIAL      = self.get_parameter('trial').value
        self.FILENAME   = self.get_parameter('filename').value
        
        self.trial_cnt      = 0
        self.tag_actual_pos = None
        self.tag_chan_pos   = []
        self.tag_newton_pos = []
        self.data = {'data': []}

        self.actual_pos_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.Position3D,
            topic       = 'TAG_0/actual_position',
            callback    = self.actualPosSubCallback,
            qos_profile = 1000
        )

        self.chan_pos_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.Position3D,
            topic       = 'server/TAG_0/pos_chan',
            callback    = self.chanPosSubCallback,
            qos_profile = 1000
        )

        self.newton_pos_sub = self.create_subscription(
            msg_type    = uwbloc_interfaces.Position3D,
            topic       = 'server/TAG_0/pos_newton',
            callback    = self.newtonPosSubCallback,
            qos_profile = 1000
        )

        self.terminator_sub = self.create_subscription(
            msg_type    = std_msgs.UInt8,
            topic       = 'TAG_0/terminator',
            callback    = self.terminatorSubCallback,
            qos_profile = 1000
        )



    def actualPosSubCallback(self, msg:uwbloc_interfaces.Position3D) -> None:
        if self.trial_cnt == 0:
            self.tag_actual_pos = list(msg.pos_3d).copy()
        
        self.trial_cnt += 1
        
        if self.trial_cnt == self.TRIAL:
            self.trial_cnt = 0

            self.data['data'].append({
                'actual_pos': self.tag_actual_pos.copy(),
                'chan_pos': self.tag_chan_pos.copy(),
                'newton_pos': self.tag_newton_pos.copy()
            })

            self.tag_actual_pos.clear()
            self.tag_chan_pos.clear()
            self.tag_newton_pos.clear()



    def chanPosSubCallback(self, msg:uwbloc_interfaces.Position3D) -> None:
        if not any(np.isnan(msg.pos_3d)):
            self.tag_chan_pos.append(list(msg.pos_3d))


    
    def newtonPosSubCallback(self, msg:uwbloc_interfaces.Position3D) -> None:
        if not any(np.isnan(msg.pos_3d)):
            self.tag_newton_pos.append(list(msg.pos_3d))



    def terminatorSubCallback(self, msg:std_msgs.UInt8) -> None:
        with open(self.FILENAME, 'w') as file:
            yaml.safe_dump(self.data, file)