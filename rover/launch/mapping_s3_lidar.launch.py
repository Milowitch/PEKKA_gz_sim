from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import subprocess

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    sllidar_ros2_path = get_package_share_directory('sllidar_ros2')
    rover_path = get_package_share_directory('rover')
    rviz_config_file = os.path.join(rover_path, 'rviz', 'nav2_default_view.rviz') # nav2_default_view mapping_s3_lidar
    slam_toolbox_params_file = os.path.join(sllidar_ros2_path, 'config', 'slam_toolbox_params.yaml')

    slam_params_file = os.path.join(rover_path, 'config', 'mapper_params_online_async.yaml')
    params_file = os.path.join(rover_path, 'config', 'nav2_params.yaml')

    ukf_config = os.path.join(rover_path, 'config', 'ukf.yaml')
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_s3_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover_description'),
                    'launch',
                    'rover.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # Node(
        #     package='ros2_laser_scan_matcher',
        #     executable='laser_scan_matcher',
        #     name='odometry_publisher',
        #     parameters=[{
        #         'base_frame': 'base_frame',
        #         'odom_frame': 'odom', 
        #         'laser_frame': 'sensor_laser', 
        #         'publish_odom': '/odom_matcher',
        #         'publish_tf': True, 
        #     }],
        # ),

        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[ukf_config],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='log'
        ),
    ])