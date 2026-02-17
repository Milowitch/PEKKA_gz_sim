
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

    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'd456_dual_camera_launch.py'
            ])
        ])
    )

    rgbd_sync0 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync0',
        output='screen',
        parameters=[{
            'queue_size': 10,
            'approx_sync': True,
            'compressed': False
        }],
        remappings=[
            ('rgb/image', 'rover/D456_1/color/image_raw'),
            ('depth/image', 'rover/D456_1/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', 'rover/D456_1/color/camera_info'),
            ('rgbd_image', 'rgbd_image0'),
        ]
    )

    rgbd_sync1 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync1',
        output='screen',
        parameters=[{
            'queue_size': 10,
            'approx_sync': True,
            'compressed': False
        }],
        remappings=[
            ('rgb/image', 'rover/D456_2/color/image_raw'),
            ('depth/image', 'rover/D456_2/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', 'rover/D456_2/color/camera_info'),
            ('rgbd_image', 'rgbd_image1'),
        ]
    )

    parameters={
        'frame_id': 'D456_1_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': True,
        'subscribe_scan_cloud': False,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'queue_size': 10,
        'wait_for_transform': 0.5,
        # RTAB-Map's internal parameters are strings:
        'Vis/BundleAdjustment': '1',
        'Vis/FeatureType': '8',
        'Vis/MaxFeatures': '1000',
        'Vis/MinDepth': '0.1',
        'Vis/MaxDepth': '10.0',
        'Vis/MinInliers': '20',
        'Vis/EstimationType': '0',
        'ORB/EdgeThreshold': '19',
        'ORB/FirstLevel': '0',
        'ORB/Gpu': 'false',
        'ORB/NLevels': '3',
        'ORB/PatchSize': '31',
        'ORB/ScaleFactor': '2',
        'ORB/ScoreType': '0',
        'ORB/WTA_K': '2',
        'KAZE/Threshold': '0.0002',
        # 'Reg/Force3DoF': 'true',
        # 'Reg/RepeatOnce': 'true',
        # 'Reg/Strategy': '1',
    }

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        parameters=[{
            'rgbd_cameras': 2,
            # RTAB-Map's internal parameters are strings:
            'Odom/KalmanMeasurementNoise': '0.01',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/Strategy': '1',
            'Odom/ResetCountdown': '1',
            'Odom/ValidDepthRatio': 'false',
            'Odom/Holonomic': '0.8',
            'Odom/KeyFrameThr': '0.6',
        },parameters]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[{
            'rgbd_cameras': 2,
            'odom_sensor_sync': True,
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.1',
            'RGBD/CreateOccupancyGrid': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '100',
            'RGBD/ForceOdom3DoF': 'true',  # Use IMU for 3DoF localization
            'ImuFilter/ComplementaryBiasAlpha': '0.98',  # IMU complementary filter bias alpha
            'ImuFilter/MadgwickGain': '0.5',  # Madgwick filter gain
            'ImuFilter/MadgwickZeta': '0.1',  # Madgwick filter Zeta

            'Grid/MaxObstacleHeight': '0.1',                # เพิกเฉยสิ่งกีดขวางที่สูงเกิน (เมตร)
            'Grid/RangeMin': '0',
            'Grid/RangeMax': '0',
            'Grid/FootprintHeight': '0.7',
            'Grid/FootprintLength': '0.7',
            'Grid/FootprintWidth': '0.5',
            'Grid/Sensor': '2', #Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'false',
        },parameters],
        remappings=[
            # ('imu', '/rover/D456_1/imu'),
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
            'frame_id': 'D456_1_link',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        sim_time,
        deskewing,
        realsense2_camera,
        rgbd_sync0,
        rgbd_sync1,
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz
    ])