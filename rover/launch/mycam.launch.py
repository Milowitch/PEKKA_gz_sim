import launch
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    rover_path = get_package_share_directory('rover')
    name_0 = 'usb_cam_0'
    # name_0_parama_path = os.path.expanduser('~/ros2_ws/src/rover_ros2/rover/config/params_1.yaml')
    name_0_parama_path = os.path.join(rover_path, 'config', 'params_1.yaml')
    name_1 = 'usb_cam_1'
    # name_1_parama_path = os.path.expanduser('~/ros2_ws/src/rover_ros2/rover/config/params_2.yaml')
    name_1_parama_path = os.path.join(rover_path, 'config', 'params_2.yaml')

    if not os.path.isfile(name_0_parama_path):
        raise FileNotFoundError(f"Parameter file {name_0_parama_path} does not exist")
    if not os.path.isfile(name_1_parama_path):
        raise FileNotFoundError(f"Parameter file {name_1_parama_path} does not exist")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=f'{name_0}',
            parameters=[name_0_parama_path],
            remappings=[
                ('image_raw', f'{name_0}/image_raw'),
                ('image_raw/compressed', f'{name_0}/image_compressed'),
                ('image_raw/compressedDepth', f'{name_0}/compressedDepth'),
                ('image_raw/theora', f'{name_0}/image_raw/theora'),
                ('camera_info', f'{name_0}/camera_info'),
            ],
            respawn=True,  # Add this line to enable auto-restart
            respawn_delay=5  # Optional: delay in seconds before restarting the node
        ),
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=f'{name_1}',
            parameters=[name_1_parama_path],
            remappings=[
                ('image_raw', f'{name_1}/image_raw'),
                ('image_raw/compressed', f'{name_1}/image_compressed'),
                ('image_raw/compressedDepth', f'{name_1}/compressedDepth'),
                ('image_raw/theora', f'{name_1}/image_raw/theora'),
                ('camera_info', f'{name_1}/camera_info'),
            ],
            respawn=True,  # Add this line to enable auto-restart
            respawn_delay=5  # Optional: delay in seconds before restarting the node
        ),
    ])
