from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def generate_launch_description():

    frame_id = 'base_footprint' # base_footprint
    odom_frame_id = 'odom'
    subscribe_depth = False
    subscribe_rgbd = False
    subscribe_scan_cloud = True
    subscribe_odom_info = True
    approx_sync = False
    sync_queue_size = 20
    topic_queue_size = 20
    wait_for_transform = 0.8
    expected_update_rate = 15.0
    Icp_VoxelSize = '0.1'
    Icp_Epsilon = '0.001'
    Icp_PointToPlaneK = '10'
    Icp_PointToPlaneRadius = '0'
    Icp_PointToPlane = 'true'
    Icp_Iterations = '10'
    Icp_MaxTranslation = '2'
    Icp_MaxCorrespondenceDistance = '1.5'
    Icp_Strategy = '1'
    Icp_OutlierRatio = '0.7'
    Icp_CorrespondenceRatio = '0.05'
    Odom_Strategy = '0'
    Odom_ScanKeyFrameThr = '0.5'
    OdomF2M_ScanSubtractRadius = '0.2'
    OdomF2M_ScanMaxSize = '21600'
    OdomF2M_BundleAdjustment = 'false'
    Odom_ResetCountdown = '15'
    Odom_GuessSmoothingDelay = '0'
    Rtabmap_StartNewMapOnLoopClosure = 'true'
    RGBD_CreateOccupancyGrid = 'false'
    Rtabmap_CreateIntermediateNodes = 'true'
    RGBD_LinearUpdate = '0.1'
    RGBD_AngularUpdate = '0.05'
    RGBD_ProximityMaxGraphDepth = '0'
    RGBD_ProximityPathMaxNeighbors = '1'
    Mem_NotLinkedNodesKept = 'false'
    Mem_STMSize = '30'
    Mem_LaserScanNormalK = '10'
    Reg_Strategy = '1'
    Reg_MaxCorrespondenceDistance = '0.2'
    Reg_MinInliers = '10'
    Odom_ICP_MaxIterations = '50'
    Odom_ICP_Epsilon = '0.0001'

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    lidar_deskewing = DeclareLaunchArgument(
        'deskewing', default_value='true',
        description='Enable lidar deskewing')
    
    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            ])
        ]),
        launch_arguments={
            'ros-args': '--log-level WARN',
        }.items(),
    )
        
    # Nodes to launch
    # ICP Odometry Node
    icp_odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen', name='icp_odometry',
        parameters=[{
            'frame_id': frame_id,  # Adjust according to your TF tree
            'odom_frame_id': odom_frame_id,
            'wait_for_transform': wait_for_transform,
            'expected_update_rate': expected_update_rate,
            'deskewing': deskewing,
            'use_sim_time': use_sim_time,
            'approx_sync': False,
            'sync_queue_size': sync_queue_size,
            'topic_queue_size': topic_queue_size,
            'Icp/VoxelSize': Icp_VoxelSize,  # Adjust voxel size to match sensor resolution
            'Icp/Epsilon': Icp_Epsilon,
            'Icp/PointToPlaneK': Icp_PointToPlaneK,  # Lower neighbors for faster computation
            'Icp/PointToPlaneRadius': Icp_PointToPlaneRadius,  # If the radius fits the L1's range
            'Icp/PointToPlane': Icp_PointToPlane,
            'Icp/Iterations': Icp_Iterations,  # Higher iterations for more robust matching
            'Icp/MaxTranslation': Icp_MaxTranslation,
            'Icp/MaxCorrespondenceDistance': Icp_MaxCorrespondenceDistance,  # Allowable matching distance
            'Icp/Strategy': Icp_Strategy,
            'Icp/OutlierRatio': Icp_OutlierRatio,
            'Icp/CorrespondenceRatio': Icp_CorrespondenceRatio,  # Adjust based on point density
            'Odom/Strategy': Odom_Strategy,
            'Odom/ScanKeyFrameThr': Odom_ScanKeyFrameThr,
            'OdomF2M/ScanSubtractRadius': OdomF2M_ScanSubtractRadius,
            'OdomF2M/ScanMaxSize': OdomF2M_ScanMaxSize,  # L1 typically provides more points
            'OdomF2M/BundleAdjustment': OdomF2M_BundleAdjustment
        }],
        remappings=[
            ('scan_cloud', 'front_3d_lidar/lidar_points')  # Ensure correct LiDAR topic
        ],
        arguments=["--ros-args", "--log-level", "WARN"]
    )
        
    rtabmap_util = Node(
        package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        parameters=[{
            'max_clouds':10,
            'fixed_frame_id':'',
            'use_sim_time':use_sim_time,
        }],
        remappings=[
            ('cloud', 'odom_filtered_input_scan')
        ],
        arguments=["--ros-args", "--log-level", "WARN"]
    )
    
    astra_pro = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('astra_camera'),
                'launch',
                'astra_pro.launch.xml'  # Use XML file here
            ])
        ]),
        launch_arguments={
            'camera_frame_id': frame_id,
            # 'point_cloud_qos': "reliable",
            # 'color_qos': "reliable",
            # 'color_camera_info_qos': "reliable",
            # 'depth_qos': "reliable",
            # 'depth_camera_info_qos': "reliable",
            # 'ir_qos': "reliable",
            # 'ir_camera_info_qos': "reliable"
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
          'frame_id': frame_id,
          'odom_frame_id': odom_frame_id,
          'subscribe_depth':subscribe_depth,
          'subscribe_rgbd':subscribe_rgbd,
          'subscribe_odom_info':subscribe_odom_info,
          'approx_sync': approx_sync,
          'sync_queue_size': sync_queue_size,
          'topic_queue_size': topic_queue_size,
          # RTAB-Map's parameters should all be string type:
          'Odom/Strategy': Odom_Strategy,
          'Odom/ResetCountdown': Odom_ResetCountdown,
          'Odom/GuessSmoothingDelay': Odom_GuessSmoothingDelay,
          'Rtabmap/StartNewMapOnLoopClosure': Rtabmap_StartNewMapOnLoopClosure,
          'RGBD/CreateOccupancyGrid':RGBD_CreateOccupancyGrid,
          'Rtabmap/CreateIntermediateNodes': Rtabmap_CreateIntermediateNodes,
          'RGBD/LinearUpdate': RGBD_LinearUpdate,
          'RGBD/AngularUpdate': RGBD_AngularUpdate,
        }],
        remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')
        ],
    )

    parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'subscribe_depth': subscribe_depth,
            'subscribe_rgbd': subscribe_rgbd,
            'subscribe_scan_cloud': subscribe_scan_cloud,
            'approx_sync': approx_sync,
            'sync_queue_size': sync_queue_size,
            'topic_queue_size': topic_queue_size,
            'wait_for_transform': wait_for_transform,
            'use_sim_time': use_sim_time,
            'Odom/Strategy': Odom_Strategy,  # Enable combining odometry with visual information
            'Odom/ResetCountdown': Odom_ResetCountdown,
            'Odom/GuessSmoothingDelay': Odom_GuessSmoothingDelay,
            'Rtabmap/StartNewMapOnLoopClosure': Rtabmap_StartNewMapOnLoopClosure,
            'Rtabmap/CreateIntermediateNodes': Rtabmap_CreateIntermediateNodes,
            'RGBD/ProximityMaxGraphDepth': RGBD_ProximityMaxGraphDepth,
            'RGBD/ProximityPathMaxNeighbors': RGBD_ProximityPathMaxNeighbors,
            'RGBD/LinearUpdate': RGBD_LinearUpdate,
            'RGBD/AngularUpdate': RGBD_AngularUpdate,
            'RGBD/CreateOccupancyGrid': RGBD_CreateOccupancyGrid,
            'Mem/NotLinkedNodesKept': Mem_NotLinkedNodesKept,
            'Mem/STMSize': Mem_STMSize,
            'Mem/LaserScanNormalK': Mem_LaserScanNormalK,
            'Reg/Strategy': Reg_Strategy,
            'Reg/MaxCorrespondenceDistance': Reg_MaxCorrespondenceDistance,
            'Reg/MinInliers': Reg_MinInliers,
            'Odom/ICP/MaxIterations': Odom_ICP_MaxIterations,
            'Odom/ICP/Epsilon': Odom_ICP_Epsilon,
            'Icp/VoxelSize': Icp_VoxelSize,
            'Icp/PointToPlaneK': Icp_PointToPlaneK,
            'Icp/PointToPlaneRadius': Icp_PointToPlaneRadius,
            'Icp/PointToPlane': Icp_PointToPlane,
            'Icp/Iterations': Icp_Iterations,
            'Icp/Epsilon': Icp_Epsilon,
            'Icp/MaxTranslation': Icp_MaxTranslation,
            'Icp/MaxCorrespondenceDistance': Icp_MaxCorrespondenceDistance,
            'Icp/Strategy': Icp_Strategy,
            'Icp/OutlierRatio': Icp_OutlierRatio,
            'Icp/CorrespondenceRatio': Icp_CorrespondenceRatio,
    }]

    remappings=[
            ('scan_cloud', 'assembled_cloud'),
            ('imu', 'unitree/imu'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
    ]

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen', name='rtabmap_slam',
        parameters=parameters,
        remappings=remappings,
    )
    
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=parameters,
        remappings=remappings,
    )
    
    return LaunchDescription([
        sim_time,
        lidar_deskewing,
        # unitree_lidar,
        icp_odometry,
        rtabmap_util,
        # astra_pro,
        # rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz,
    ])