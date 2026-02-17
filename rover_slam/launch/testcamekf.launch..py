
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
    mapping = LaunchConfiguration('mapping')
    file = LaunchConfiguration('file')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    deskewing = DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing')
    
    declare_mapping = DeclareLaunchArgument(
        'mapping', default_value="'true'",
        description='true is Mapping false is Localization')
    declare_file = DeclareLaunchArgument(
        'file', default_value="'~/maps/mymap_0.db'",
        description='~/maps/mymap_0.db')

    realsense2_camera_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'd456_1_imu_launch.py'
            ])
        ])
    )

    realsense2_camera_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'd456_2_imu_launch.py'
            ])
        ])
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace='',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'queue_size': 10,
            'approx_sync': True,
            'compressed': False
        }],
        remappings=[
            ('rgb/image', '/D456_1/color/image_raw'),
            ('depth/image', '/D456_1/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/D456_1/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
        ]
    )

    parameters={
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'approx_sync_max_interval': 0.03,
        'rgbd_cameras': 1,
        # RTAB-Map's internal parameters are strings:
        'Vis/BundleAdjustment': '1',
        'Vis/FeatureType': '8',
        'Vis/MaxFeatures': '0',
        'Vis/MinDepth': '0.2',
        'Vis/MaxDepth': '8.0',
        'Vis/MinInliers': '50',
        'ORB/EdgeThreshold': '15',
        'ORB/FirstLevel': '0',
        'ORB/Gpu': 'false',
        'ORB/NLevels': '4',
        'ORB/PatchSize': '25',
        'ORB/ScaleFactor': '2',
        'ORB/ScoreType': '0',
        'ORB/WTA_K': '2',
        'KAZE/Threshold': '0.0002',
        'LoopGPS':'1',
        'Reg/Strategy': '0',
        'Optimizer/Strategy': '2',
        'Optimizer/PriorsIgnored': 'false', 
    }

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        namespace='',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            # RTAB-Map's internal parameters are strings:
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
            ('odom', '/rgbd_odom')
        ]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[os.path.join(get_package_share_directory("rover_slam"), 'params', 'dual_ekf.yaml')],  #ekf_camera
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        namespace='',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '100',
            'RGBD/ForceOdom3DoF': 'true',  # Use IMU for 3DoF localization
            'ImuFilter/ComplementaryBiasAlpha': '0.98',  # IMU complementary filter bias alpha
            'ImuFilter/MadgwickGain': '0.5',  # Madgwick filter gain
            'ImuFilter/MadgwickZeta': '0.1',  # Madgwick filter Zeta

            'Grid/MaxGroundAngle': '40.0',
            'Grid/MaxObstacleHeight': '0.5',                # เพิกเฉยสิ่งกีดขวางที่สูงเกิน (เมตร)
            'Grid/RangeMin': '0',
            'Grid/RangeMax': '4',
            'Grid/MinClusterSize': '50',
            'Grid/NormalK': '20',
            # 'Grid/FootprintHeight': '0.7',
            # 'Grid/FootprintLength': '0.7',
            # 'Grid/FootprintWidth': '0.5',
            'Grid/Sensor': '1', #Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',

            # 'GridGlobal/FootprintRadius': '0.40',
            'GridGlobal/OccupancyThr': '0.60',

            'Mem/IncrementalMemory': mapping, # false is localization true is mapping
            'database_path': file,
        },parameters],
        remappings=[
            # ('imu', '/unitree/imu'),
            ('imu', '/D456_1/imu/data_filtered'),
            ('gps/fix', '/ublox_gps_node/fix'),
        ],
        arguments=[
            # '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ]
    )

    # RTAB-Map Viz Node
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        namespace='',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
        }],
    )

    base_footprint_tf = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        respawn=True,
        respawn_delay=1,
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

    base_link_tf_1 = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        respawn=True,
        respawn_delay=1,
        arguments=[
            "0.29", # translation_x
            "0.0", # translation_y
            "0.425", # translation_z
            "0.0",  # rotation_yaw
            "0.0349",# rotation_pitch
            "0.0", # rotation_roll
            "base_link",  # parant
            "D456_1_link",   # child
        ]
    )

    base_link_tf_2 = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        respawn=True,
        respawn_delay=1,
        arguments=[
            "0.0", # translation_x
            "0.0", # translation_y
            "0.28", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "base_link",  # parant
            "D456_1_link",   # child
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
            "gps",   # child
        ]
    )
     
    return LaunchDescription([
        sim_time,
        deskewing,
        declare_mapping,
        declare_file,
        base_footprint_tf,
        realsense2_camera_1,
        base_link_tf_1,
        realsense2_camera_2,
        base_link_tf_2,
        rgbd_sync,
        rgbd_odometry,
        #ekf,
        gps_tf,
        rtabmap_slam,
        rtabmap_viz
    ])