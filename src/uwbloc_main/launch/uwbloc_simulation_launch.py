from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import json


SCALED_SPEED_OF_LIGHT       = 299792.458
SCALE_RATIO                 = 1000.0
SYNC_PERIOD                 = 0.35
TAG_BROADCAST_PERIOD        = 0.05
ANCHOR_CALIBRATION_PERIOD   = 0.15
ANCHOR_CALIBRATION_CNT      = 40
SERVER_CALL_PERIOD          = 0.4
NETWORK_INFO = {
    'master_clock': [2.5, 2.5, 0.2],
    'tag_list': [
        ['TAG_0', [3.0, 4.5, 0.8], 2*np.pi*0.5, 4.2],
        ['TAG_1', [1.0, 1.0, 0.3], 2*np.pi*0.8, 3.1]
    ],
    'anchor_list': [
        ['ANCHOR_0', [0.0, 0.0, 0.0], 0.01],
        ['ANCHOR_1', [5.0, 0.0, -0.2], 0.01],
        ['ANCHOR_2', [6.545084972, 4.755282581, 0.2], 0.01],
        ['ANCHOR_3', [2.5, 7.694208842, -0.1], 0.01],
        ['ANCHOR_4', [-1.545084972, 4.755282581, 0.3], 0.01]
    ]
}


def generate_launch_description():
    nodes = []

    nodes.append(
        Node(
            package     = 'uwbloc_simulations',
            executable  = 'medium',
            name        = 'MEDIUM',
            parameters  = [
                {'scaled_speed_of_light': SCALED_SPEED_OF_LIGHT},
                {'scale_ratio': SCALE_RATIO},
                {'network_info': json.dumps(NETWORK_INFO)}
            ]
        )
    )

    nodes.append(
        Node(
            package     = 'uwbloc_simulations',
            executable  = 'tdoa_server',
            name        = 'TDOA_SERVER',
            parameters  = [
                {'scaled_speed_of_light': SCALED_SPEED_OF_LIGHT},
                {'scale_ratio': SCALE_RATIO},
                {'network_info': json.dumps(NETWORK_INFO)},
                {'server_call_period': SERVER_CALL_PERIOD}
            ]
        )
    )

    nodes.append(
        Node(
            package     = 'uwbloc_simulations',
            executable  = 'master_clock',
            name        = 'MASTER_CLOCK',
            parameters  = [
                {'position': NETWORK_INFO['master_clock']},
                {'sync_period': SYNC_PERIOD}
            ]
        )
    )

    for tag in NETWORK_INFO['tag_list']:
        nodes.append(
            Node(
                package     = 'uwbloc_simulations',
                executable  = 'tag',
                name        = tag[0],
                parameters  = [
                    {'id': tag[0]},
                    {'broadcast_period': TAG_BROADCAST_PERIOD},
                    {'initial_position': tag[1]},
                    {'moving_omega': tag[2]},
                    {'moving_radius': tag[3]}
                ]
            )
        )

    for anchor in NETWORK_INFO['anchor_list']:
        nodes.append(
            Node(
                package     = 'uwbloc_simulations',
                executable  = 'anchor',
                name        = anchor[0],
                parameters  = [
                    {'scaled_speed_of_light': SCALED_SPEED_OF_LIGHT},
                    {'scale_ratio': SCALE_RATIO},
                    {'id': anchor[0]},
                    {'position': anchor[1]},
                    {'time_drift': anchor[2]},
                    {'calib_period': ANCHOR_CALIBRATION_PERIOD},
                    {'calib_cnt': ANCHOR_CALIBRATION_CNT}
                ]
            )
        )

    return LaunchDescription(nodes)