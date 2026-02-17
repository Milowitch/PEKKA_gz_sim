
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os

def generate_launch_description():
    package_share_directory = os.path.join(get_package_share_directory('rover_slam'), 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    deskewing = DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing')

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        parameters=[{
            'frame_id': 'dm_base_frame',  # Astra Pro frame
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgbd': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'sync_queue_size': 30,
            'topic_queue_size': 10,
            'approx_sync_max_interval': 0.02,
            # RTAB-Map's internal parameters are strings:
            'Vis/FeatureType': 'SURF',
            'Vis/MaxFeatures': '5000',
            'Vis/MinDepth': '0.2',
            'Vis/MaxDepth': '2.0',
            'Vis/MinInliers': '5',
            'Odom/KalmanMeasurementNoise': '0.5',
            'Odom/KalmanProcessNoise': '0.1',
            'Odom/Strategy': '0',
            'Vis/BundleAdjustment': '0',
        }],
        remappings=[
            ('rgb/camera_info', '/apc/left/camera_info'),
            ('rgb/image', '/apc/left/image_color'),
            ('depth/image', '/apc/depth/image_raw'),
        ],
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[{
            'frame_id': 'dm_base_frame',  # Astra Pro frame
            'subscribe_depth': True,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'sync_queue_size': 30,
            'topic_queue_size': 10,
            'approx_sync_max_interval': 0.02,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.2',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/CreateOccupancyGrid': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '60',
        }],
        remappings=[
            ('rgb/camera_info', '/apc/left/camera_info'),
            ('rgb/image', '/apc/left/image_color'),
            ('depth/image', '/apc/depth/image_raw'),
        ],
        arguments=[
            '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ]
    )

    # RTAB-Map Viz Node
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'dm_base_frame',
            'odom_frame_id': 'odom',
            'subscribe_odom_info': True,
            'subscribe_depth': True,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'sync_queue_size': 30,
            'topic_queue_size': 10,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('rgb/camera_info', '/apc/left/camera_info'),
            ('rgb/image', '/apc/left/image_color'),
            ('depth/image', '/apc/depth/image_raw'),
        ]
    )
    
    return LaunchDescription([
        sim_time,
        deskewing,
        # apc_camera,
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz
    ])