from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory of the parameter file
    package_share_directory = os.path.join(get_package_share_directory('rover_slam'))

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    lidar_deskewing = DeclareLaunchArgument(
        'deskewing', default_value='false',
        description='Enable lidar deskewing')

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
            # RTAB-Map's internal parameters are strings:
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '30',
            'Icp/VoxelSize': '0.7',  # Adjust according to your lidar's resolution
            'Icp/Epsilon': '0',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '10000',
            'OdomF2M/BundleAdjustment': 'false'
        }],
        remappings=[
          ('scan_cloud', 'rover/unitree/lidar_points')  # Remap lidar points
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
          'frame_id': 'base_footprint',  # Use lidar frame
          'subscribe_depth': True,
          'subscribe_rgb': False,
          'subscribe_scan_cloud': True,
          'approx_sync': True,
          'wait_for_transform': 0.5,
          'use_sim_time': use_sim_time,
          # RTAB-Map's internal parameters are strings:
          'RGBD/ProximityMaxGraphDepth': '0',
          'RGBD/ProximityPathMaxNeighbors': '1',
          'RGBD/AngularUpdate': '0.05',
          'RGBD/LinearUpdate': '0.05',
          'RGBD/CreateOccupancyGrid': 'false',
          'Mem/NotLinkedNodesKept': 'false',
          'Mem/STMSize': '30',
          'Mem/LaserScanNormalK': '20',
          'Reg/Strategy': '1',
          'Icp/VoxelSize': '0.7',  # Adjust according to your lidar's resolution
          'Icp/PointToPlaneK': '20',
          'Icp/PointToPlaneRadius': '0',
          'Icp/PointToPlane': 'true',
          'Icp/Iterations': '30',
          'Icp/Epsilon': '0.001',
          'Icp/MaxTranslation': '3',
          'Icp/MaxCorrespondenceDistance': '1',
          'Icp/Strategy': '1',
          'Icp/OutlierRatio': '0.7',
          'Icp/CorrespondenceRatio': '0.2'
        }],
        remappings=[
          ('scan_cloud', 'assembled_cloud'),
          ('imu', 'rover/unitree/imu'),
          ('gps/fix', 'rover/gps/rawdata'),
          ('rgb/camera_info', 'rover/D456/color/camera_info'),
          ('rgb/image', 'rover/D456/color/image_raw'),
          ('depth/image', 'rover/D456/depth/points')
        ],
        arguments=[
          '-d'  # This will delete the previous database (~/.ros/rtabmap.db)
        ])
  
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[{
          'frame_id': 'base_footprint',  # Match lidar's frame
          'odom_frame_id': 'odom',
          'subscribe_odom_info': True,
          'subscribe_scan_cloud': True,
          'approx_sync': False,
          'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ])
    
    return LaunchDescription([
        sim_time,
        lidar_deskewing,
        icp_odometry,
        rtabmap_util,
        rtabmap_slam,
        rtabmap_viz
    ])