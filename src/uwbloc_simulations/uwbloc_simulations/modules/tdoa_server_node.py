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
        self.TAG_NUM        = len(self.TAG_LIST)
        self.ANCHOR_NUM     = len(self.ANCHOR_LIST)

        self.anchor_set     = set(())
        self.anchor_data    = dict(())
        self.tag_tdoa       = dict(())

        self.newton_guess   = np.array([
            [3.0],
            [4.0],
            [3.0]
        ])

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

        self.tag_pos_chan_pub   = []
        self.tag_pos_newton_pub = []
        for tag in self.TAG_LIST:
            self.tag_pos_chan_pub.append(
                self.create_publisher(
                    msg_type    = uwbloc_interfaces.Position3D,
                    topic       = f'server/{tag[0]}/pos_chan',
                    qos_profile = 1000
                )
            )
            self.tag_pos_newton_pub.append(
                self.create_publisher(
                    msg_type    = uwbloc_interfaces.Position3D,
                    topic       = f'server/{tag[0]}/pos_newton',
                    qos_profile = 1000
                )
            )



    def calcTDOA(self) -> None:
        self.tag_tdoa.clear()

        for tag in self.TAG_LIST:
            self.tag_tdoa.update({tag[0]: []})
            mean_toa    = []

            for anchor in self.ANCHOR_LIST:
                global_ts   = self.anchor_data[anchor[0]]['sync_time'].copy()
                local_ts    = self.anchor_data[anchor[0]]['machine_time'].copy()
                data        = self.anchor_data[anchor[0]]['tag_data'][tag[0]][1].copy()
                lerp        = lambda x_t, x, y : y[0] + ((y[1] - y[0])/(x[1] - x[0]))*(x_t - x[0])
                num         = float(len(data))
                mean        = 0.0

                for val in data:
                    # mean += lerp(val, local_ts, global_ts)
                    mean += val
                mean /= num

                mean_toa.append(mean)

            for i in range(self.ANCHOR_NUM):
                self.tag_tdoa[tag[0]].append(mean_toa[i] - mean_toa[0])



    def calcChanPositioning(self, tdoa:list) -> list:
        # construct G1 and h1
        G1 = []
        h1 = []
        for i in range(self.ANCHOR_NUM):
            if i == 0: continue
            
            r = (tdoa[i]*1e-6)*self.SCALED_C
            d = self.ANCHOR_LIST[i][1][0]**2.0 + self.ANCHOR_LIST[i][1][1]**2.0 + self.ANCHOR_LIST[i][1][2]**2.0

            row = self.ANCHOR_LIST[i][1] + [r]
            G1.append(row)
            h1.append([r**2.0 - d])

        G1 = -2.0*np.array(G1)
        h1 = np.array(h1)

        # construct Q
        Q = np.eye(self.ANCHOR_NUM - 1)

        # calculate p1_hat
        p1_hat = np.linalg.pinv(G1.T @ Q @ G1) @ G1.T @ Q @ h1
        
        # calculate B1
        B1 = []
        for i in range(self.ANCHOR_NUM):
            if i == 0: continue
            
            row         = [0.0 for __ in range(self.ANCHOR_NUM - 1)]
            row[i - 1]  = np.sqrt(
                (self.ANCHOR_LIST[i][1][0] - p1_hat[0].item())**2.0 +
                (self.ANCHOR_LIST[i][1][1] - p1_hat[1].item())**2.0 +
                (self.ANCHOR_LIST[i][1][2] - p1_hat[2].item())**2.0
            )
            B1.append(row)

        B1 = np.array(B1)

        # calculate W1 and p1
        W1 = (0.25/(self.SCALED_C**2.0))*(np.linalg.pinv(B1 @ Q @ B1))
        p1 = np.linalg.pinv(G1.T @ W1 @ G1) @ G1.T @ W1 @ h1

        # construct G2, h2, B2, and S
        G2 = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 1, 1]
        ])
        h2 = np.array([
            [p1[0].item()**2.0],
            [p1[1].item()**2.0],
            [p1[2].item()**2.0],
            [p1[3].item()**2.0]
        ])
        B2 = np.array([
            [p1[0].item(), 0, 0, 0],
            [0, p1[1].item(), 0, 0],
            [0, 0, p1[2].item(), 0],
            [0, 0, 0, p1[3].item()]
        ])
        S = np.array([
            [np.sign(p1[0].item()), 0, 0],
            [0, np.sign(p1[1].item()), 0],
            [0, 0, np.sign(p1[2].item())]
        ])

        # calculate W2 and p2
        W2 = 0.25*(np.linalg.pinv(B2) @ G1.T @ W1 @ G1 @ np.linalg.pinv(B2))
        p2 = np.linalg.pinv(G2.T @ W2 @ G2) @ G2.T @ W2 @ h2

        # calculate final result
        p = S @ np.sqrt(p2)

        return [p[0].item(), p[1].item(), p[2].item()]



    def newtonNablaF(self, xk:np.ndarray, tdoa:list) -> np.ndarray:
        calc_norm = lambda x, idx: np.sqrt(
            (x[0].item() - self.ANCHOR_LIST[idx][1][0])**2.0 + 
            (x[1].item() - self.ANCHOR_LIST[idx][1][1])**2.0 + 
            (x[2].item() - self.ANCHOR_LIST[idx][1][2])**2.0
        )

        fx = 0.0
        fy = 0.0
        fz = 0.0

        for i in range(self.ANCHOR_NUM):
            if i == 0: continue

            r   = (tdoa[i]*1e-6)*self.SCALED_C
            x   = xk[0].item()
            y   = xk[1].item()
            z   = xk[2].item()
            xo  = self.ANCHOR_LIST[0][1][0]
            yo  = self.ANCHOR_LIST[0][1][1]
            zo  = self.ANCHOR_LIST[0][1][2]
            xi  = self.ANCHOR_LIST[i][1][0]
            yi  = self.ANCHOR_LIST[i][1][1]
            zi  = self.ANCHOR_LIST[i][1][2]
            di  = calc_norm(xk, i)
            do  = calc_norm(xk, 0)

            fx += (di - do - r)*((x - xi)/di - (x - xo)/do)
            fy += (di - do - r)*((y - yi)/di - (y - yo)/do)
            fz += (di - do - r)*((z - zi)/di - (z - zo)/do)

        return 2.0*np.array([
            [fx],
            [fy],
            [fz]
        ])
    


    def newtonNumericalNablaF(self, xk:np.ndarray, tdoa:list) -> np.ndarray:
        calc_norm = lambda x, idx: np.sqrt(
            (x[0].item() - self.ANCHOR_LIST[idx][1][0])**2.0 + 
            (x[1].item() - self.ANCHOR_LIST[idx][1][1])**2.0 + 
            (x[2].item() - self.ANCHOR_LIST[idx][1][2])**2.0
        )

        eps_x = np.array([
            [0.0001],
            [0.0],
            [0.0]
        ])
        eps_y = np.array([
            [0.0],
            [0.0001],
            [0.0]
        ])
        eps_z = np.array([
            [0.0],
            [0.0],
            [0.0001]
        ])

        fx = 0.0
        fy = 0.0
        fz = 0.0

        for i in range(self.ANCHOR_NUM):
            if i == 0: continue

            r = (tdoa[i]*1e-6)*self.SCALED_C
            x = xk[0].item()
            y = xk[1].item()
            z = xk[2].item()

            fx += ((calc_norm(xk + eps_x, i) - calc_norm(xk + eps_x, 0) - r)**2.0 - (calc_norm(xk, i) - calc_norm(xk, 0) - r)**2.0)/0.0001
            fy += ((calc_norm(xk + eps_y, i) - calc_norm(xk + eps_y, 0) - r)**2.0 - (calc_norm(xk, i) - calc_norm(xk, 0) - r)**2.0)/0.0001
            fz += ((calc_norm(xk + eps_z, i) - calc_norm(xk + eps_z, 0) - r)**2.0 - (calc_norm(xk, i) - calc_norm(xk, 0) - r)**2.0)/0.0001

        return np.array([
            [fx],
            [fy],
            [fz]
        ])



    def newtonJacobianF(self, xk:np.ndarray, tdoa:list) -> np.ndarray:
        calc_norm = lambda x, idx: np.sqrt(
            (x[0].item() - self.ANCHOR_LIST[idx][1][0])**2.0 + 
            (x[1].item() - self.ANCHOR_LIST[idx][1][1])**2.0 + 
            (x[2].item() - self.ANCHOR_LIST[idx][1][2])**2.0
        )

        dxx = 0.0
        dyy = 0.0
        dzz = 0.0
        dxy = 0.0
        dxz = 0.0
        dyz = 0.0

        for i in range(self.ANCHOR_NUM):
            if i == 0: continue

            r       = (tdoa[i]*1e-6)*self.SCALED_C
            x       = xk[0].item()
            y       = xk[1].item()
            z       = xk[2].item()
            diffxo  = x - self.ANCHOR_LIST[0][1][0]
            diffyo  = y - self.ANCHOR_LIST[0][1][1]
            diffzo  = z - self.ANCHOR_LIST[0][1][2]
            diffxi  = x - self.ANCHOR_LIST[i][1][0]
            diffyi  = y - self.ANCHOR_LIST[i][1][1]
            diffzi  = z - self.ANCHOR_LIST[i][1][2]
            do      = calc_norm(xk, 0)
            di      = calc_norm(xk, i)
            do2     = do*do
            di2     = di*di
            do3     = do2*do
            di3     = di2*di

            dxx += ((diffxi)/(di) - (diffxo)/(do))**2.0 + (di - do - r)*((di2 - diffxi**2.0)/(di3) - (do2 - diffxo**2.0)/(do3))
            dyy += ((diffyi)/(di) - (diffyo)/(do))**2.0 + (di - do - r)*((di2 - diffyi**2.0)/(di3) - (do2 - diffyo**2.0)/(do3))
            dzz += ((diffzi)/(di) - (diffzo)/(do))**2.0 + (di - do - r)*((di2 - diffzi**2.0)/(di3) - (do2 - diffzo**2.0)/(do3))
            
            dxy += ((diffxi)/(di) - (diffxo)/(do))*((diffyi)/(di) - (diffyo)/(do)) - (di - do - r)*((diffxi*diffyi)/(di3) - (diffxo*diffyo)/(do3))
            dxz += ((diffxi)/(di) - (diffxo)/(do))*((diffzi)/(di) - (diffzo)/(do)) - (di - do - r)*((diffxi*diffzi)/(di3) - (diffxo*diffzo)/(do3))
            dyz += ((diffyi)/(di) - (diffyo)/(do))*((diffzi)/(di) - (diffzo)/(do)) - (di - do - r)*((diffyi*diffzi)/(di3) - (diffyo*diffzo)/(do3))

        res = 2.0*np.array([
            [dxx, dxy, dxz],
            [dxy, dyy, dyz],
            [dxz, dyz, dzz]
        ])

        return res
    


    def newtonNumericalJacobianF(self, xk:np.ndarray, tdoa:list) -> np.ndarray:
        epsilon_x = np.array([
            [0.00001],
            [0.0],
            [0.0]
        ])
        epsilon_y = np.array([
            [0.0],
            [0.00001],
            [0.0]
        ])
        epsilon_z = np.array([
            [0.0],
            [0.0],
            [0.00001]
        ])

        dx = (self.newtonNablaF(xk + epsilon_x, tdoa) - self.newtonNablaF(xk, tdoa))*100000
        dy = (self.newtonNablaF(xk + epsilon_y, tdoa) - self.newtonNablaF(xk, tdoa))*100000
        dz = (self.newtonNablaF(xk + epsilon_z, tdoa) - self.newtonNablaF(xk, tdoa))*100000

        res = np.column_stack((dx, dy, dz))

        return res



    def calcNewtonPositioning(self, tdoa:list) -> list:
        # pos = self.newton_guess.copy()
        
        # for i in range(5):
        #     nF  = self.newtonNumericalNablaF(pos, tdoa)
        #     JF  = self.newtonNumericalJacobianF(pos, tdoa)
        #     pos = pos - np.linalg.pinv(JF) @ nF

        # self.newton_guess = pos.copy()

        # self.get_logger().info(f'\nNewton :\n{pos}')

        # construct G1 and h1
        G1 = []
        h1 = []
        for i in range(self.ANCHOR_NUM):
            if i == 0: continue
            
            r = (tdoa[i]*1e-6)*self.SCALED_C
            d = self.ANCHOR_LIST[i][1][0]**2.0 + self.ANCHOR_LIST[i][1][1]**2.0 + self.ANCHOR_LIST[i][1][2]**2.0

            row = self.ANCHOR_LIST[i][1] + [r]
            G1.append(row)
            h1.append([r**2.0 - d])

        G1 = -2.0*np.array(G1)
        h1 = np.array(h1)

        # construct Q
        Q = np.eye(self.ANCHOR_NUM - 1)

        # calculate p1_hat
        p1_hat = np.linalg.pinv(G1.T @ Q @ G1) @ G1.T @ Q @ h1

        return [p1_hat[0].item(), p1_hat[1].item(), p1_hat[2].item()]



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
            self.calcTDOA()

            for i in range(self.TAG_NUM):
                chan_msg    = uwbloc_interfaces.Position3D()
                newton_msg  = uwbloc_interfaces.Position3D()
                
                chan_msg.pos_3d     = self.calcChanPositioning(self.tag_tdoa[self.TAG_LIST[i][0]])
                newton_msg.pos_3d   = self.calcNewtonPositioning(self.tag_tdoa[self.TAG_LIST[i][0]])

                self.tag_pos_chan_pub[i].publish(chan_msg)
                self.tag_pos_newton_pub[i].publish(newton_msg)


    
    def serverTimerCallback(self) -> None:
        pub_msg         = uwbloc_interfaces.ServerCall()
        pub_msg.data    = 101

        self.server_call_pub.publish(pub_msg)

        self.anchor_set.clear()
        self.anchor_data.clear()