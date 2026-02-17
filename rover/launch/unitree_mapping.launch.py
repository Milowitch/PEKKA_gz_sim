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

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    lidar_deskewing = DeclareLaunchArgument(
        'deskewing', default_value='true',
        description='Enable lidar deskewing')
    
    rover_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('rover_description'),
                'launch',
                'rover.launch.py'
            ])
        ]),
        launch_arguments={
            'ros-args': '--log-level WARN',
        }.items(),
    )
    
    # astra_pro = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             get_package_share_directory('astra_camera'),
    #             'launch',
    #             'astra_pro.launch.xml'  # Use XML file here
    #         ])
    #     ]),
    #     launch_arguments={
    #         'camera_frame_id': 'lidar_link',
    #         # 'point_cloud_qos': "reliable",
    #         # 'color_qos': "reliable",
    #         # 'color_camera_info_qos': "reliable",
    #         # 'depth_qos': "reliable",
    #         # 'depth_camera_info_qos': "reliable",
    #         # 'ir_qos': "reliable",
    #         # 'ir_camera_info_qos': "reliable"
    #         'ros-args': '--log-level WARN',
    #     }.items(),
    # )

    # RGBD Odometry Node
    # rgbd_odometry = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     output='screen',
    #     name='rgbd_odometry',
    #     parameters=[{
    #       'frame_id': 'base_footprint',
    #       'odom_frame_id': 'odom',
    #       'subscribe_depth':True,
    #       'subscribe_rgbd':False,
    #       'subscribe_odom_info':True,
    #       'approx_sync': True,
    #       'sync_queue_size': 20,
    #       'topic_queue_size': 20,
    #       # RTAB-Map's parameters should all be string type:
    #       'RGBD/LinearUpdate': '0.1',
    #       'RGBD/AngularUpdate': '0.05',
    #     }],
    #     remappings=[
    #       ('rgb/image', '/camera/color/image_raw'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/camera/depth/image_raw')
    #     ],
    # )
    
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

    parameters={
        'frame_id': 'base_footprint',  # Adjust according to your TF tree
        'odom_frame_id': 'odom',
        'wait_for_transform': 0.8,
        'use_sim_time': use_sim_time,
        'scan_voxel_size': 0.1,  # Adjust voxel size to match sensor resolution
        'scan_normal_k': 10,  # Lower neighbors for faster computation
        'scan_range_max': 30.0,
        'scan_range_min': 0.05,
        'Icp/Iterations': '10',  # Higher iterations for more robust matching
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneRadius': '0',  # If the radius fits the L1's range
        'Icp/MaxTranslation': '2',
        'Icp/MaxCorrespondenceDistance': '1.5',  # Allowable matching distance
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.05',  # Adjust based on point density
        'Odom/ScanKeyFrameThr': '0.5',
        'OdomF2M/ScanSubtractRadius': '0.2',
        'OdomF2M/ScanMaxSize': '21600',  # L1 typically provides more points
        'OdomF2M/BundleAdjustment': 'false'
    }
        
    # ICP Odometry Node
    icp_odometry = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen', name='icp_odometry',
        parameters=[{
            'expected_update_rate': 15.0,
            'deskewing': deskewing,
        }, parameters],
        remappings=[
            ('scan_cloud', 'unitree/lidar_points')  # Ensure correct LiDAR topic
        ],
        arguments=[
            '-d',  # Clear previous database
            "--ros-args", "--log-level", "WARN"
        ]
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
        arguments=[
            '-d',  # Clear previous database
            "--ros-args", "--log-level", "WARN"
        ]
    )
        
    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen', name='rtabmap_slam',
        parameters=[{
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'RGBD/ProximityPathMaxNeighbors': '0',
            'Grid/Sensor': '0',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Mem/LaserScanNormalK': '10',  # Adjust based on L1 point density
            'Reg/Strategy': '1',
        }, parameters],
        remappings=[
            ('scan_cloud', 'assembled_cloud')
        ],
        arguments=[
            '-d',  # Clear previous database
            "--ros-args", "--log-level", "WARN"
        ]
    )
    
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[{
            'frame_id':'base_footprint',
            'odom_frame_id':'odom',
            'subscribe_odom_info':True,
            'subscribe_scan_cloud':True,
            'approx_sync':False,
            'use_sim_time':use_sim_time,
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ],
        arguments=[
            '-d',  # Clear previous database
            "--ros-args", "--log-level", "WARN"
        ]
    )
    
    return LaunchDescription([
        sim_time,
        lidar_deskewing,
        unitree_lidar,
        icp_odometry,
        rtabmap_util,
        # astra_pro,
        # rgbd_odometry,
        rtabmap_slam,
        rtabmap_viz,
        rover_description,
    ])