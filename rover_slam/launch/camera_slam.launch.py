
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

    astra_pro = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('astra_camera'),
                'launch',
                'astra_pro.launch.xml'  # Use XML file here
            ])
        ]),
        launch_arguments={
            'camera_frame_id': 'camera_link',
            'ros-args': '--log-level WARN',
        }.items(),
    )

    remappings=[
        ('rgb/camera_info', 'rover/camera/color/camera_info'),
        ('rgb/image', 'rover/camera/color/image_raw'),
        ('depth/image', 'rover/camera/depth/image_raw'),
    ]

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        parameters=[{
            'frame_id': 'camera_link',  # Astra Pro frame
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgbd': False,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'sync_queue_size': 50,
            'topic_queue_size': 20,
            'approx_sync_max_interval': 0.05,
            # RTAB-Map's internal parameters are strings:
            'Vis/FeatureType': '8',
            'Vis/MaxFeatures': '5000',
            'Vis/MinDepth': '0',
            'Vis/MaxDepth': '0',
            'Vis/MinInliers': '20',
            'ORB/EdgeThreshold': '19',
            'ORB/FirstLevel': '0',
            'ORB/Gpu': 'false',
            'ORB/NLevels': '3',
            'ORB/PatchSize': '31',
            'ORB/ScaleFactor': '2',
            'ORB/ScoreType': '0',
            'ORB/WTA_K': '2',
            'Odom/KalmanMeasurementNoise': '0.01',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/Strategy': '0',
            'Vis/BundleAdjustment': '1',
        }],
        remappings=remappings,
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',  # Astra Pro frame
            'subscribe_depth': True,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'sync_queue_size': 50,
            'topic_queue_size': 20,
            'approx_sync_max_interval': 0.05,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.5',
            'RGBD/AngularUpdate': '0.2',
            'RGBD/CreateOccupancyGrid': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '100',
        }],
        remappings=remappings,
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
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'subscribe_odom_info': True,
            'subscribe_depth': True,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': False,
            'approx_sync': True,
            'sync_queue_size': 50,
            'topic_queue_size': 20,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
        }],
        remappings=remappings
    )
    
    return LaunchDescription([
        sim_time,
        deskewing,
        # astra_pro,
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz
    ])