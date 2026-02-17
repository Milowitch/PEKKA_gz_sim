
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
import yaml

def generate_launch_description():
    rover_path = get_package_share_directory('rover')
    name_0 = 'right'
    # name_0_parama_path = os.path.expanduser('~/ros2_ws/src/rover_ros2/rover/config/params_1.yaml')
    name_0_parama_path = os.path.join(rover_path, 'config', 'params_1.yaml')
    name_1 = 'left'
    # name_1_parama_path = os.path.expanduser('~/ros2_ws/src/rover_ros2/rover/config/params_2.yaml')
    name_1_parama_path = os.path.join(rover_path, 'config', 'params_2.yaml')

    if not os.path.isfile(name_0_parama_path):
        raise FileNotFoundError(f"Parameter file {name_0_parama_path} does not exist")
    if not os.path.isfile(name_1_parama_path):
        raise FileNotFoundError(f"Parameter file {name_1_parama_path} does not exist")
    
    rover_slam_path = get_package_share_directory('rover_slam')
    disparity_param_path = os.path.join(rover_slam_path, 'params', 'disparity.yaml')
    with open(disparity_param_path, 'r') as f:
        disparity_params = yaml.safe_load(f)
    point_cloud_param_path = os.path.join(rover_slam_path, 'params', 'point_cloud.yaml')
    with open(point_cloud_param_path, 'r') as f:
        point_cloud_params = yaml.safe_load(f)

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    # Launch arguments
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    deskewing = DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing')
    
    cam0 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'{name_0}',
        parameters=[name_0_parama_path],
        remappings=[
            ('image_raw', f'rover/camera/{name_0}/image_raw'),
            ('image_raw/compressed', f'rover/camera/{name_0}/image_compressed'),
            ('image_raw/compressedDepth', f'rover/camera/{name_0}/compressedDepth'),
            ('image_raw/theora', f'rover/camera/{name_0}/image_raw/theora'),
            ('camera_info', f'rover/camera/{name_0}/camera_info'),
        ],
        respawn=True,
        respawn_delay=1
    )
    cam1 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=f'{name_1}',
        parameters=[name_1_parama_path],
        remappings=[
            ('image_raw', f'rover/camera/{name_1}/image_raw'),
            ('image_raw/compressed', f'rover/camera/{name_1}/image_compressed'),
            ('image_raw/compressedDepth', f'rover/camera/{name_1}/compressedDepth'),
            ('image_raw/theora', f'rover/camera/{name_1}/image_raw/theora'),
            ('camera_info', f'rover/camera/{name_1}/camera_info'),
        ],
        respawn=True,
        respawn_delay=1
    )

    disparity_node = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        parameters=[disparity_params],
        remappings=[
            ('/left/image_rect', 'rover/camera/left/image_raw'),
            ('/right/image_rect', 'rover/camera/right/image_raw'),
            ('/left/camera_info', 'rover/camera/left/camera_info'),
            ('/right/camera_info', 'rover/camera/right/camera_info'),
        ],
        output='screen',
    )
    
    point_cloud_node = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        parameters=[point_cloud_params],
        remappings=[
            ('/left/image_rect_color', 'rover/camera/left/image_raw'),
            ('/left/camera_info', 'rover/camera/left/camera_info'),
            ('/right/camera_info', 'rover/camera/right/camera_info'),
        ],
        output='screen',
    )

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
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '5000',
            'Vis/MinDepth': '0.5',
            'Vis/MaxDepth': '8.0',
            'Vis/MinInliers': '10',
            'ORB/EdgeThreshold': '19',
            'ORB/FirstLevel': '0',
            'ORB/Gpu': 'false',
            'ORB/NLevels': '8',
            'ORB/PatchSize': '31',
            'ORB/ScaleFactor': '1.1',
            'ORB/ScoreType': 'HARRIS_SCORE',
            'ORB/WTA_K': '4',
            'Odom/KalmanMeasurementNoise': '0.8',
            'Odom/KalmanProcessNoise': '0.2',
            'Odom/Strategy': '0',
            'Vis/BundleAdjustment': '0',
        }],
        remappings=[
            ('rgb/camera_info', 'rover/camera/color/camera_info'),
            ('rgb/image', 'rover/camera/color/image_raw'),
            ('depth/image', 'rover/camera/depth/image_raw'),
        ],
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
        remappings=[
            ('rgb/camera_info', 'rover/camera/color/camera_info'),
            ('rgb/image', 'rover/camera/color/image_raw'),
            ('depth/image', 'rover/camera/depth/image_raw'),
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
        remappings=[
            ('rgb/camera_info', 'rover/camera/color/camera_info'),
            ('rgb/image', 'rover/camera/color/image_raw'),
            ('depth/image', 'rover/camera/depth/image_raw'),
        ]
    )
    
    return LaunchDescription([
        sim_time,
        deskewing,
        cam0, # right
        cam1, # left
        disparity_node,
        point_cloud_node,
        # rgbd_odometry,
        # rtabmap_slam,
        # rtabmap_viz
    ])