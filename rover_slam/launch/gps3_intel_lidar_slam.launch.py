
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
    
    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            ])
        ])
    )

    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'd456_1_launch.py'
            ])
        ])
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
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
            ('rgbd_image', 'rgbd_image'),
        ]
    )

    rtabmap_util = Node(
        package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        parameters=[{
            'max_clouds': 10,
            'fixed_frame_id': 'lidar_link',
            'use_sim_time': use_sim_time,
            'range_min': 1.0,
            # 'range_max': 30.0,
        }],
        remappings=[
            ('cloud', 'rover/unitree/lidar_points')
        ]
    )

    parameters={
        'frame_id': 'D456_1_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'rgbd_cameras': 1,
        # RTAB-Map's internal parameters are strings:
        'Vis/BundleAdjustment': '1',
        'Vis/FeatureType': '8',
        'Vis/MaxFeatures': '0',
        'Vis/MinDepth': '0.1',
        # 'Vis/MaxDepth': '10.0',
        'Vis/MinInliers': '30',
    
        'ORB/EdgeThreshold': '19',
        'ORB/FirstLevel': '0',
        'ORB/Gpu': 'false',
        'ORB/NLevels': '3',
        'ORB/PatchSize': '31',
        'ORB/ScaleFactor': '2',
        'ORB/ScoreType': '0',
        'ORB/WTA_K': '2',

        'KAZE/Threshold': '0.0002',

        'Optimizer/Strategy': '2',
        'Optimizer/Robust': 'false',
        'Optimizer/PriorsIgnored': 'false',

        'Reg/Strategy': '0',
    }

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        parameters=[{
            # RTAB-Map's internal parameters are strings:
            'Odom/Strategy': '1',
            'Odom/FilteringStrategy': '1',
            'Odom/KalmanMeasurementNoise': '0.01',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/ResetCountdown': '1',
            'Odom/ValidDepthRatio': 'false',
            'Odom/Holonomic': 'true',
            'Odom/AlignWithGround': 'true',
        },parameters],
        arguments=[
            '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[{
            'subscribe_scan_cloud': True,
            'odom_sensor_sync': True,
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/ForceOdom3DoF': 'true',

            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '100',
            'ImuFilter/ComplementaryBiasAlpha': '0.98',  # IMU complementary filter bias alpha
            'ImuFilter/ComplementaryDoAdpativeGain': 'true',
            'ImuFilter/MadgwickGain': '0.5',  # Madgwick filter gain
            'ImuFilter/MadgwickZeta': '0.1',  # Madgwick filter Zeta

            'Icp/RangeMin': '1.0',
            # 'Icp/RangeMax': '30.0',
            'Icp/VoxelSize': '0.05',
            'Icp/PointToPlaneK': '0',
            'Icp/PointToPlaneRadius': '0.1',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '1.5',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/OutlierRatio': '0.6',
            'Icp/CorrespondenceRatio': '0.02',
        },parameters],
        remappings=[
            ('scan_cloud', 'assembled_cloud'),
            ('imu', 'rover/D456_1/imu'),
            # ('imu', 'rover/unitree/imu'),
            ('gps/fix', 'gps/rawdata'),
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

    lidar_tf = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.175", # translation_x
            "-0.04", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "D456_1_link",  # parant
            "lidar_link",   # child
        ]
    )

    gps_tf = Node(
        name="gps_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.48", # translation_x
            "-0.015", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "D456_1_link",  # parant
            "gps_link",   # child
        ]
    )
    
    return LaunchDescription([
        sim_time,
        deskewing,
        # unitree_lidar,
        # realsense2_camera,
        lidar_tf,
        gps_tf,
        rtabmap_util,
        rgbd_sync,
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz
    ])