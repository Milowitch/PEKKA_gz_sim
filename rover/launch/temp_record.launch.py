import launch

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from datetime import datetime
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    rover_path = get_package_share_directory('rover')
    # Define the path to the 'bags' directory in the home directory
    bags_dir = os.path.expanduser('~/maps')
    # Check if the directory exists, and if not, create it
    if not os.path.exists(bags_dir):
        os.makedirs(bags_dir)
    # Generate the timestamp string
    timestamp = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
    # Define the bag file name with the timestamp
    temp_filename = os.path.expanduser(f'~/maps/bag_temp_{timestamp}')
    nuc_filename = os.path.expanduser(f'~/maps/bag_nuc_{timestamp}')

    namespace = 'rover'
    all_log_level = 'fatal'
    log_level = 'info'

    rosbag_temp = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', temp_filename, '/rover_health'],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )

    rosbag_nuc = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', nuc_filename, '/diagnostics'],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )

    return launch.LaunchDescription([
        rosbag_temp,
        rosbag_nuc,
    ])