
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    rover_description = os.path.join(get_package_share_directory('rover_description'), 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    lidar_deskewing = DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing')
    
    ld_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_s3_launch.py'
            ])
        ])
    )
    
    parameters={
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_for_transform': 0.5,
        'use_sim_time': use_sim_time,
        # RTAB-Map's internal parameters are strings:
        'Icp/Strategy': '1',    #ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/RangeMin': '1.0',
        'Icp/RangeMax': '100.0',
        'Icp/VoxelSize': '0',
        'Icp/PointToPlaneK': '50',
        'Icp/PointToPlaneRadius': '0',
        'Icp/PointToPlane': 'false',
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '1.5',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/OutlierRatio': '0.6',
        'Icp/CorrespondenceRatio': '0.02',

        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '1',
        'Optimizer/Strategy': '1',
        'Optimizer/PriorsIgnored': 'true',
        'Optimizer/Iterations': '20',
        'Optimizer/Epsilon': '0.0',
    }
        
    # ICP Odometry Node
    icp_odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[{
            'expected_update_rate': 10.0,
            'scan_cloud_is_2d': True,
            'subscribe_scan_cloud': False,
            'deskewing': deskewing,
            'Odom/FilteringStrategy': '1',
            'Odom/KalmanMeasurementNoise': '0.01',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/Strategy': '1',
            'Odom/ResetCountdown': '1',
            'Odom/ValidDepthRatio': 'false',
            'Odom/Holonomic': 'true',
            'Odom/AlignWithGround': 'true',
        },parameters],
        remappings=[
            # ('scan_cloud', 'assembled_cloud')
        ]
    )


    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[{
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'subscribe_rgb': False,
            'odom_sensor_sync': False,
            # RTAB-Map's internal parameters are strings:
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '200',
            'Mem/LaserScanNormalK': '50',
            'ImuFilter/ComplementaryBiasAlpha': '0.98',  # IMU complementary filter bias alpha
            'ImuFilter/ComplementaryDoAdpativeGain': 'true',
            'ImuFilter/MadgwickGain': '0.5',  # Madgwick filter gain
            'ImuFilter/MadgwickZeta': '0.1',  # Madgwick filter Zeta

            'RGBD/NeighborLinkRefining': 'true',

            'Grid/MaxObstacleHeight': '2.0',                # เพิกเฉยสิ่งกีดขวางที่สูงเกิน (เมตร)
            'Grid/RangeMin': '0.0',
            'Grid/RangeMax': '100.0',
            'Grid/MinClusterSize': '5',
            'Grid/NormalK': '5',
            'Grid/FootprintHeight': '0.7',
            'Grid/FootprintLength': '0.7',
            'Grid/FootprintWidth': '0.5',
            'Grid/Sensor': '0', #Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
            'Grid/CellSize': '0.05',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',

            'Mem/IncrementalMemory': 'true', # false is localization true is mapping
        },parameters],
        remappings=[
            # ('scan_cloud', 'assembled_cloud'),
            ('imu', 'rover/D456_1/imu/rawdata'),
            # ('imu', 'rover/unitree/imu'),
            # ('gps/fix', 'gps/rawdata'),
        ],
        arguments=[
            '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ]
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
        }]
    )
    
    base_footprint_tf = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0", # translation_x
            "0.0", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "base_footprint",  # parant
            "base_link",   # child
        ]
    )

    base_link_tf = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0", # translation_x
            "0.0", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "base_link",  # parant
            "lidar_link",   # child
        ]
    )
    
    return LaunchDescription([
        sim_time,
        lidar_deskewing,
        ld_lidar,
        base_footprint_tf,
        base_link_tf,
        icp_odometry,
        rtabmap_slam,
        rtabmap_viz
    ])