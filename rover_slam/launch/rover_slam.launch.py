
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
    dual_ekf_params_file = '/home/station/dual_ekf.yaml'

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
        'frame_id': 'D456_1_link',
        'odom_frame_id': 'odom',
        'subscribe_odom': True,
        'subscribe_depth': False,
        'subscribe_rgbd': True,  
        #'odom_sensor_sync': True, 
        #'subscribe_odom_info': True,
        # Use EKF odometry
        
        'rgb_topic': '/D456_1/color/image_raw',
        'depth_topic': '/D456_1/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/D456_1/color/camera_info',
        'approx_sync': True,
        'approx_sync_max_interval': 0.03,#########################3333333333
        #'rgbd_cameras': 1,
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
            ('odom', '/rgbd_odom'),
            ('rgb/image', '/D456_1/color/image_raw'),
            ('depth/image', '/D456_1/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/D456_1/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
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

    # EKF node for odometry
    EKF_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[dual_ekf_params_file],
            remappings=[
                ('odometry/filtered', 'odometry/local')
            ]
    )

    # EKF node for map
    EKF_node = Node (
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[dual_ekf_params_file],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
    )

    # NavSat transform node
    NavSat_transform =  Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[dual_ekf_params_file],
            remappings=[
                ('imu/data', '/D456_1/imu/data_filtered'),
                ('gps/fix', '/gps/rawdata'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odom')
            ]
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
            "-0.0", # translation_x
            "-0.0", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "D456_1_link",  # parant
            "gps_link",   # child
        ]
    )
    compass_tf = Node(
        name="compass_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.0", # translation_x
            "-0.0", # translation_y
            "0.0", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "D456_1_link",  # parant
            "compass",   # child
        ]
    )
    map_tf =    Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="swri_transform",
    arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        namespace='',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            # Base params
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_rgbd': True,
            'rgbd_sync': True,
            'rgb_topic': '/D456_1/color/image_raw',
            'depth_topic': '/D456_1/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/D456_1/color/camera_info',
            'odom_topic': '/odom',
            'imu_topic': '/D456_1/imu/data_filtered',
            'approx_sync': True,
            'approx_rgbd_sync': True,
            'qos': 2,
            'queue_size': 30,
            'sync_queue_size': 30,
            'visual_odometry': False,
            'guess_odom_frame_id': 'odom',
            'args': "--delete_db_on_start --Reg/Force3DoF true",

            # Extended RTAB-Map config
            'approx_sync_max_interval': 0.03,
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
            'LoopGPS': '1',
            'Reg/Strategy': '0',
            'Optimizer/Strategy': '2',
            'Optimizer/PriorsIgnored': 'false',
            'RGBD/ProximityMaxGraphDepth': '5',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '100',
            'RGBD/ForceOdom3DoF': 'true',
            'ImuFilter/ComplementaryBiasAlpha': '0.98',
            'ImuFilter/MadgwickGain': '0.5',
            'ImuFilter/MadgwickZeta': '0.1',
            'Grid/MaxGroundAngle': '40.0',
            'Grid/MaxObstacleHeight': '0.5',
            'Grid/RangeMin': '0',
            'Grid/RangeMax': '4',
            'Grid/MinClusterSize': '50',
            'Grid/NormalK': '20',
            'Grid/Sensor': '1',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Grid/RayTracing': 'true',
            'GridGlobal/OccupancyThr': '0.60',

            # External variables assumed to be set before (e.g., via Python logic)
            'Mem/IncrementalMemory': mapping,  # True for SLAM, False for localization
            'database_path': file,             # Set to your DB path
        }]
    )
    rtabmap_viz = Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            name='rtabmap_viz',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'rgbd_sync': True,
                'depth_topic': '/D456_1/aligned_depth_to_color/image_raw',
                'rgb_topic': '/D456_1/color/image_raw',
                'camera_info_topic': '/D456_1/color/camera_info',
                'odom_topic': '/odom',
                'imu_topic': '/D456_1/imu/data_filtered',
                'approx_sync': True,
                'approx_rgbd_sync': True,
                'qos': 2,
                'queue_size': 30,
                'sync_queue_size': 30,
                'visual_odometry': False,
                'guess_odom_frame_id': 'odom',
            }]
        )  
    return LaunchDescription([
        sim_time,
        deskewing,
        declare_mapping,
        declare_file,
        base_footprint_tf,
        #realsense2_camera_1,
        #base_link_tf_1,
        realsense2_camera_2,
        base_link_tf_1,
        rgbd_sync,
        rgbd_odometry,
        #ekf,
        gps_tf,
        compass_tf,
        EKF_odom ,
        EKF_node ,
        NavSat_transform ,
        map_tf,
        #rtabmap_slam,
        #rtabmap_viz,
    ])
'''
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
    config_path = os.path.join(get_package_share_directory('rover_slam'), 'params', 'rtabmap.cfg')

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    lidar_deskewing = DeclareLaunchArgument(
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
            'camera_frame_id': 'base_footprint',
            'ros-args': '--log-level WARN',
        }.items(),
    )

    # RGBD Odometry Node
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        name='rgbd_odometry',
        parameters=[{
            'frame_id': 'base_footprint',  # Astra Pro frame
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
            'Vis/MinInliers': '10',
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
            'Reg/Strategy': '1',
        }],
        remappings=[
            ('rgb/camera_info', 'rover/camera/color/camera_info'),
            ('rgb/image', 'rover/camera/color/image_raw'),
            ('depth/image', 'rover/camera/depth/image_raw'),
        ],
    )
    
    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            ])
        ])
    )
        
    # ICP Odometry Node
    icp_odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[
          {
            'frame_id': 'base_footprint',  # Match the lidar's frame
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.5,
            'expected_update_rate': 15.0,
            'deskewing': deskewing,
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'sync_queue_size': 50,
            'topic_queue_size': 20,
            'approx_sync_max_interval': 0.05,
            # RTAB-Map's internal parameters are strings:
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '20',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            # 'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '5000',
            'OdomF2M/BundleAdjustment': 'false',
            'Odom/Strategy': '0',
            'Reg/Strategy': '1',
        }],
        remappings=[
          ('scan_cloud', 'rover/unitree/lidar_points')
        ])

    rtabmap_util = Node(
        package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        parameters=[{
          'max_clouds': 10,
          'fixed_frame_id': '',
          'use_sim_time': use_sim_time,
        }],
        remappings=[
          ('cloud', 'odom_filtered_input_scan')
        ])
        
    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap_slam', output='screen',
        parameters=[{
          'frame_id': 'base_footprint',
          'odom_frame_id': 'odom',
          'subscribe_depth': True,
          'subscribe_scan_cloud': True,
          'subscribe_odom_info': True,
          'subscribe_stereo': False,
          'subscribe_scan': False,
          'subscribe_rgb': False,
          'subscribe_user_data': False,
          'odom_sensor_sync': True,
          'map_always_update': False,
          'approx_sync': True,
          'sync_queue_size': 50,
          'topic_queue_size': 20,
          'approx_sync_max_interval': 0.05,
          'wait_for_transform': 0.5,
          'use_sim_time': use_sim_time,
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
          'Vis/BundleAdjustment': '1',
          'RGBD/ProximityMaxGraphDepth': '5',
          'RGBD/ProximityPathMaxNeighbors': '2',
          'RGBD/LinearUpdate': '0.5',
          'RGBD/AngularUpdate': '0.2',
          'RGBD/CreateOccupancyGrid': 'true',
          'Mem/NotLinkedNodesKept': 'false',
          'Mem/STMSize': '100',
          'Mem/LaserScanNormalK': '20',
          'Icp/PointToPlane': 'true',
          'Icp/Iterations': '30',
          'Icp/VoxelSize': '0.1',
          'Icp/Epsilon': '0.001',
          'Icp/PointToPlaneK': '20',
          'Icp/PointToPlaneRadius': '0',
          'Icp/MaxTranslation': '3',
          'Icp/MaxCorrespondenceDistance': '1',
          'Icp/OutlierRatio': '0.7',
          'Icp/CorrespondenceRatio': '0.01',
          'Reg/Strategy': '1',
        }],
        remappings=[
          ('scan_cloud', 'assembled_cloud'),
          # ('scan_cloud', 'rover/unitree/lidar_points'),
          ('imu', 'rover/unitree/imu'),
          ('rgb/camera_info', 'rover/camera/color/camera_info'),
          ('rgb/image', 'rover/camera/color/image_raw'),
          ('depth/image', 'rover/camera/depth/image_raw'),
        ],
        arguments=[
          '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ])
  
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[{
          'frame_id': 'base_footprint',
          'odom_frame_id': 'odom',
          'subscribe_scan_cloud': True,
          'subscribe_depth': True,
          'subscribe_odom_info': True,
          'subscribe_rgb': False,
          'approx_sync': True,
          'sync_queue_size': 50,
          'topic_queue_size': 20,
          'wait_for_transform': 0.5,
          'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan'),
            ('imu', 'rover/unitree/imu'),
            ('rgb/camera_info', 'rover/camera/color/camera_info'),
            ('rgb/image', 'rover/camera/color/image_raw'),
            ('depth/image', 'rover/camera/depth/image_raw'),
        ])
    
    return LaunchDescription([
        sim_time,
        lidar_deskewing,
        # astra_pro,
        # rgbd_odometry,
        # unitree_lidar,
        icp_odometry,
        rtabmap_util,
        rtabmap_slam,
        rtabmap_viz
    ])'''