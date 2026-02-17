import launch
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from datetime import datetime

def generate_launch_description():
    rover_path = get_package_share_directory('rover')

    # Define the path to the 'bags' directory in the home directory
    bags_dir = os.path.expanduser('~/bags')
    # Check if the directory exists, and if not, create it
    if not os.path.exists(bags_dir):
        os.makedirs(bags_dir)
    # Generate the timestamp string
    timestamp = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
    # Define the bag file name with the timestamp
    bag_filename = os.path.expanduser(f'~/bags/bag_station_{timestamp}')

    all_log_level = 'fatal'
    log_level = 'info'
    
    station = launch_ros.actions.Node(
        package='rover',
        executable='station',
        name='station_node',
        respawn=True,  # Add this line to enable auto-restart
        respawn_delay=1,  # Optional: delay in seconds before restarting the node
        parameters=[{
            'js': 2,
        }]
    )
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node_1',
        respawn=True,  
        respawn_delay=1,
        remappings=[
            ('/joy', '/joy_station'),
            ('/joy/set_feedback', '/joy_station/set_feedback')
        ],
        parameters=[{
            'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            'qos_overrides./parameter_events.publisher.history': 'keep_last',
            'qos_overrides./parameter_events.publisher.durability': 'volatile',
            'qos_overrides./parameter_events.publisher.depth': 1,
        }],
    )

    station_lora = launch_ros.actions.Node(
        package='rover',
        executable='station_lora',
        name='station_lora_node',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'send_rate': 2.5,
            'target_addr': 0,
            'target_ch': 18,
            'lost_connect_time': 5.0
        }]
    )

    gui = launch_ros.actions.Node(
        package='rover',
        executable='station_gui',
        name='station_gui_node',
        respawn=True,
        respawn_delay=1
    )

    station_soil = launch_ros.actions.Node(
        package='rover',
        executable='station_soil',
        name='station_soil_node',
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_filename],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )

    return launch.LaunchDescription([
        station,
        joy_node,
        station_lora,
        station_soil,
        #gui,
        # rosbag,
    ])
