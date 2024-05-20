from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
import json


SCALED_SPEED_OF_LIGHT       = 299792.458
SCALE_RATIO                 = 1000.0
SYNC_PERIOD                 = 0.15
TAG_BROADCAST_PERIOD        = 0.05
ANCHOR_CALIBRATION_PERIOD   = 0.06
ANCHOR_CALIBRATION_CNT      = 40
SERVER_CALL_PERIOD          = 0.2
TRIAL_PER_POSITION          = 18
NETWORK_INFO = {
    'master_clock': [2.5, 2.5, 1.0],
    'tag_list': [
        ['TAG_0', [-3.3, 8.2], [-3.3, 8.2], 0.5, 0.17]
    ],
    'anchor_list': [
        ['ANCHOR_0', [0.0, 0.0, 0.0], 0.01],
        ['ANCHOR_1', [5.0, 0.0, -0.2], 0.01],
        ['ANCHOR_2', [6.545084972, 4.755282581, 0.2], 0.01],
        ['ANCHOR_3', [2.5, 7.694208842, -0.1], 0.01],
        ['ANCHOR_4', [-1.545084972, 4.755282581, 0.3], 0.01]
        # ['ANCHOR_0', [0.0, 0.0, 0.0], 0.01],
        # ['ANCHOR_1', [5.0, 0.0, -0.2], 0.01],
        # ['ANCHOR_2', [5.0, 5.0, 0.2], 0.01],
        # ['ANCHOR_3', [0.0, 5.0, -0.1], 0.01]
    ]
}


def generate_launch_description():
    nodes = []

    nodes.append(
        Node(
            package     = 'uwbloc_data',
            executable  = 'sim_lattice_record',
            name        = 'RECORDER',
            parameters  = [
                {'filename': '/home/dhonan/Workspace/uwb-localization/src/uwbloc_data/data/lattice_noise1.yaml'},
                {'trial': TRIAL_PER_POSITION}
            ]
        )
    )

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
                executable  = 'tag_lattice',
                name        = tag[0],
                parameters  = [
                    {'id': tag[0]},
                    {'broadcast_period': TAG_BROADCAST_PERIOD},
                    {'grid_x': tag[1]},
                    {'grid_y': tag[2]},
                    {'height': tag[3]},
                    {'step': tag[4]},
                    {'trial': TRIAL_PER_POSITION}
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