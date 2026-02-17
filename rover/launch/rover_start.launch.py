from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ublox_pkg = get_package_share_directory('ublox_gps')
    rover_slam_pkg = get_package_share_directory('rover_slam')
    rtabmap_pkg = get_package_share_directory('rtabmap_launch')
    rover_nav2_pkg = get_package_share_directory('rover_nav2')

    # ==== CLI-overridable args (เฉพาะที่จำเป็น) ====
    mapping_arg = DeclareLaunchArgument(
        'mapping', default_value='true',
        description='Enable RTAB-Map mapping (true/false)'
    )
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value=os.path.expanduser('~/maps/pim_2.db'),
        description='Path to RTAB-Map database file'
    )

    # พาธไฟล์ config เดิมของ ublox_gps
    zed_f9p_yaml = os.path.join(ublox_pkg, 'config', 'zed_f9p.yaml')

    # -- Nodes / LaunchIncludes --
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','D456_1_link']
    )

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[zed_f9p_yaml, {'device': '/dev/gps'}]  # ใช้ /dev/gps จาก udev symlink
    )

    rover_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_slam_pkg, 'launch', 'rover_slam.launch.py')
        )
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_pkg, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_viz': 'true',
            'rgbd_sync': 'true',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'depth_topic': '/D456_1/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/D456_1/color/camera_info',
            'rgb_topic': '/D456_1/color/image_raw',
            'odom_topic': '/odom',
            'guess_odom_frame_id': 'odom',
            'visual_odometry': 'false',
            'imu_topic': '/D456_1/imu/data_filtered',
            'qos': '2',
            'queue_size': '30',
            'sync_queue_size': '30',
            'namespace': '/',
            'odom_log_level': 'warn',
            'mapping': LaunchConfiguration('mapping'),
            'database_path': LaunchConfiguration('database_path'),
            'rtabmap_args': (
                "--Reg/Force3DoF true "
                "--Grid/Sensor 1 "
                "--Grid/FromDepth true "
                "--Grid/3D true "
                "--Grid/RayTracing true "
                "--Grid/CellSize 0.05 "
                "--Grid/MaxGroundAngle 40.0 "
                "--Grid/MaxObstacleHeight 2.0 "
                "--Grid/RangeMin 0.0 "
                "--Grid/RangeMax 40.0 "
                "--Grid/MinClusterSize 60 "
                "--Grid/NormalK 20"
            ),
        }.items()
    )

    rover_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_nav2_pkg, 'launch', 'rover_nav2.launch.py')
        )
    )

    # ==== Delayed schedule ====
    # หมายเหตุ: period เป็นเวลานับจากเริ่ม launch (ไม่ใช่ต่อคิวกัน)
    return LaunchDescription([
        mapping_arg,
        database_path_arg,

        # 0s: TF
        tf_static,

        # 3s: ublox_gps
        TimerAction(period=1.0, actions=[ublox_node]),

        # 5s: rover_slam
        TimerAction(period=2.0, actions=[rover_slam_launch]),

        # 8s: rtabmap
        TimerAction(period=6.0, actions=[rtabmap_launch]),

        # 10s: nav2
        TimerAction(period=9.0, actions=[rover_nav2_launch]),
    ])
