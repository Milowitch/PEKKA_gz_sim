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
    
    # Define the list of preferred adapters in order
    preferred_adapters = [
        'wlx202351dd8642',  # TP-Link Micro B
        'wlx081f7125abd0',  # TP-Link USB
        'wlo1',             # Built-in adapter
        'wlp4s0',           # Built-in adapter
        'wlp111s0f0',       # Built-in adapter
    ]

    ssid_options = ['TP-rover', 'TP-rover 1', 'TP-rover 2', 'TP-rover 3']  # Replace with your network SSID
    password = "Sl33plai3"  # Replace with your network password

    # Define the path to the 'bags' directory in the home directory
    bags_dir = os.path.expanduser('~/bags')
    # Check if the directory exists, and if not, create it
    if not os.path.exists(bags_dir):
        os.makedirs(bags_dir)
    # Generate the timestamp string
    timestamp = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
    # Define the bag file name with the timestamp
    bag_filename = os.path.expanduser(f'~/bags/bag_rover_{timestamp}')

    namespace = 'rover'
    all_log_level = 'fatal'
    log_level = 'info'
    
    robot_drive = Node(
        package='rover',
        executable='robot_drive',
        name='robot_drive_node',
        respawn=True,  # Add this line to enable auto-restart
        respawn_delay=1,  # Optional: delay in seconds before restarting the node
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    camera_symlink = Node(
        package='rover',
        executable='camera_symlink',
        name='camera_symlink_node',
        respawn=True,  # Add this line to enable auto-restart
        respawn_delay=1,  # Optional: delay in seconds before restarting the node
        output='screen',
        prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        shell=True,
    )

    robot_control = Node(
        package ='rover',
        executable='robot_control',
        name='robot_control_node',
        output='screen',
        respawn=True,  
        respawn_delay=1,
        parameters=[{
            'js': 0,
            'speed': 0.4,
            'donut_speed': 0.07,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    nav_node = Node(
        package='rover',
        executable='nav',
        name='nav_node',
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )
    robot_health = Node(
        package='rover',
        executable='robot_health',
        name='robot_health_node',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'preferred_adapters': preferred_adapters,
            'ssid_options': ssid_options,
            'password': password,
            'wifi_auto_connect': False,
        }],
        arguments=['--ros-args', '--log-level', all_log_level],
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_0',
        respawn=True,  
        respawn_delay=1,
        remappings=[
            ('/joy', '/joy_rover'),
            ('/joy/set_feedback', '/joy_rover/set_feedback')
        ],
        arguments=['--ros-args', '--log-level', all_log_level],
        parameters=[{
            'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            'qos_overrides./parameter_events.publisher.history': 'keep_last',
            'qos_overrides./parameter_events.publisher.durability': 'volatile',
            'qos_overrides./parameter_events.publisher.depth': 1,
        }],
    )
    diagnostic_common_diagnostics = Node(
        package='diagnostic_common_diagnostics',
        executable='sensors_monitor.py',
        name='sensors_monitor',
        respawn=True, 
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
        parameters=[{
            'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            'qos_overrides./parameter_events.publisher.history': 'keep_last',
            'qos_overrides./parameter_events.publisher.durability': 'volatile',
            'qos_overrides./parameter_events.publisher.depth': 1,
        }],
    )
    zero0 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_0',
        arguments=["serial", "--dev", "/dev/zero0", "-b", "115200", '--ros-args', '--log-level', all_log_level],
        respawn=True,  
        respawn_delay=1,
    )
    zero1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_1',
        arguments=["serial", "--dev", "/dev/zero1", "-b", "115200", '--ros-args', '--log-level', all_log_level],
        respawn=True,  
        respawn_delay=1,
    )
    zero2 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_2',
        arguments=["serial", "--dev", "/dev/zero2", "-b", "115200", '--ros-args', '--log-level', all_log_level],
        respawn=True,  
        respawn_delay=1,
    )
    zero3 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_3',
        arguments=["serial", "--dev", "/dev/zero3", "-b", "115200", '--ros-args', '--log-level', all_log_level],
        respawn=True,  
        respawn_delay=1,
    )
    zero4 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_4',
        arguments=["serial", "--dev", "/dev/agent", "-b", "115200", '--ros-args', '--log-level', all_log_level],
        respawn=True,  
        respawn_delay=1,
    )
    cam0 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'{name_0}',
        parameters=[name_0_parama_path,{
            'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            'qos_overrides./parameter_events.publisher.history': 'keep_last',
            'qos_overrides./parameter_events.publisher.durability': 'volatile',
            'qos_overrides./parameter_events.publisher.depth': 1,
        }],
        remappings=[
            ('image_raw', f'camera/{name_0}/image_raw'),
            ('image_raw/compressed', f'camera/{name_0}/image_compressed'),
            ('image_raw/compressedDepth', f'camera/{name_0}/compressedDepth'),
            ('image_raw/theora', f'camera/{name_0}/image_raw/theora'),
            ('camera_info', f'camera/{name_0}/camera_info'),
        ],
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )
    cam1 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'{name_1}',
        parameters=[name_1_parama_path,{
            'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
            'qos_overrides./parameter_events.publisher.history': 'keep_last',
            'qos_overrides./parameter_events.publisher.durability': 'volatile',
            'qos_overrides./parameter_events.publisher.depth': 1,
        }],
        remappings=[
            ('image_raw', f'camera/{name_1}/image_raw'),
            ('image_raw/compressed', f'camera/{name_1}/image_compressed'),
            ('image_raw/compressedDepth', f'camera/{name_1}/compressedDepth'),
            ('image_raw/theora', f'camera/{name_1}/image_raw/theora'),
            ('camera_info', f'camera/{name_1}/camera_info'),
        ],
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    robot_lora = Node(
        package='rover',
        executable='robot_lora',
        name='robot_lora_node',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'serial_port': '/dev/lora1',
            'baud_rate': 115200,
            'send_rate': 2.0,
            'target_addr': 0,
            'target_ch': 18,
            'lost_connect_time': 5.0
        }],
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    robot_send_lora = Node(
        package='rover',
        executable='robot_send_lora',
        name='robot_send_lora_node',
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    soil = Node(
        package='rover',
        executable='soil',
        name='soil_node',
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    robot_file_send = Node(
        package='rover',
        executable='robot_file_send',
        name='robot_file_send_node',
        respawn=True,
        respawn_delay=1,
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    camera_record = Node(
        package='rover',
        executable='camera_record',
        name='camera_record_node',
        respawn=True,
        respawn_delay=1,
        remappings=[
            ('image_raw/compressed0', f'camera/{name_0}/image_compressed'),
            ('image_raw/compressed1', f'camera/{name_1}/image_compressed'),
        ],
        arguments=['--ros-args', '--log-level', all_log_level],
    )

    rover_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('rover_description'),
                'launch',
                'rover.launch.py'
            ])
        ])
    )

    sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('rover_slam'),
                'launch',
                'sensor.launch.py'
            ])
        ])
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_filename],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )

    return launch.LaunchDescription([
        # robot_drive, # ไม่ใช้แล้ว
        # camera_symlink,
        robot_control,
        nav_node,
        robot_health,
        joy_node,
        diagnostic_common_diagnostics,
        # zero0, 
        zero1, # Nav
        zero2, # Health
        zero3, # Drive
        zero4, # payload
        cam0,
        cam1,
        robot_send_lora,
        robot_lora,
        soil,
        robot_file_send,
        sensor,
        # camera_record,
        # rover_description,
        # rosbag,
    ])