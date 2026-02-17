
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

    # Use the arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    mapping = LaunchConfiguration('mapping')
    file = LaunchConfiguration('file')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_mapping = DeclareLaunchArgument(
        'mapping', default_value="'true'",
        description='true is Mapping false is Localization')
    declare_file = DeclareLaunchArgument(
        'file', default_value="'~/maps/mymap_0.db'",
        description='~/maps/mymap_0.db')

    s3_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_s3_launch.py'
            ])
        ])
    )
    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'd456_1_imu_launch.py'
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
            'approx_sync_max_interval': 0.05,
            'compressed': False
        }],
        remappings=[
            ('rgb/image', '/D456_1/color/image_raw'),
            ('depth/image', '/D456_1/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/D456_1/color/camera_info'),
            ('rgbd_image', '/rgbd_image'),
        ]
    )

    parameters={
        'frame_id': 'base_link',   # ไอดีเฟรมของ กล้อง
        'use_sim_time': use_sim_time,
        'subscribe_odom_info': True,    # ใช้ Odom info
        'approx_sync': True,            # ทำการ sync ให้ข้อมูล และ time ของแต่ละเซนเซอร์ตรงกัน
        'approx_sync_max_interval': 0.05,
        # RTAB-Map's internal parameters are strings:
        'Reg/Strategy': '0',            # 0=Vis, 1=Icp, 2=VisIcp
        'Optimizer/Strategy': '2',      # Graph optimization strategy: 0=TORO, 1=g2o and 2=GTSAM
        'Optimizer/PriorsIgnored': 'true',
        'Optimizer/Iterations': '20',
        'Optimizer/Epsilon': '0.0',
    }

    parameters_rgbd={
        'subscribe_depth': False,       # ใช้ depth
        'subscribe_rgbd': True,         # ใช้ rgbd
        'rgbd_cameras': 1,              # จำนวนกล้อง rgbd
        # RTAB-Map's internal parameters are strings:
        'Vis/BundleAdjustment': '1',
        'Vis/FeatureType': '8',         # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB
        'Vis/MaxFeatures': '0',         # จำนวน Features 0=ไม่จำกัด
        'Vis/MinDepth': '0.2',          # ระยะกล้องใกล้สุด
        'Vis/MaxDepth': '8.0',          # ระยะกล้องไกลสุด
        'Vis/MinInliers': '30',         # ความสอดคล้องของคุณลักษณะขั้นต่ำในการคำนวณ/ยอมรับการแปลง
        'ORB/EdgeThreshold': '15',      # นี่คือขนาดของเส้นขอบที่ไม่พบฟีเจอร์ใดๆ ซึ่งควรจะตรงกับพารามิเตอร์ patchSize โดยประมาณ
        'ORB/FirstLevel': '0',          # ควรเป็น 0 ในการใช้งานปัจจุบัน
        'ORB/Gpu': 'false',             # การใช้ gpu
        'ORB/NLevels': '4',             # จำนวนระดับพีระมิด ระดับที่เล็กที่สุดจะมีขนาดเชิงเส้นเท่ากับ input_image_linear_size/pow(scaleFactor, nlevels)
        'ORB/PatchSize': '25',          # ขนาดของแพตช์ที่ใช้โดยตัวระบุ BRIEF แบบวางแนว แน่นอนว่าในเลเยอร์พีระมิดขนาดเล็ก พื้นที่ภาพที่รับรู้ซึ่งครอบคลุมโดยฟีเจอร์จะมีขนาดใหญ่กว่า
        'ORB/ScaleFactor': '2',         # อัตราส่วนการลดขนาดพีระมิดมากกว่า 1 scaleFactor==2 หมายถึงพีระมิดแบบคลาสสิก โดยแต่ละระดับถัดไปจะมีพิกเซลน้อยกว่าระดับก่อนหน้า 4 เท่า แต่ปัจจัยมาตราส่วนที่ใหญ่ขนาดนี้จะทำให้คะแนนการจับคู่ฟีเจอร์ลดลงอย่างมาก ในทางกลับกัน หากปัจจัยมาตราส่วนที่ใกล้ 1 เกินไป หมายความว่าคุณจะต้องใช้ระดับพีระมิดมากขึ้นเพื่อให้ครอบคลุมช่วงมาตราส่วนบางช่วง และความเร็วจะลดลง
        'ORB/ScoreType': '0',           # HARRIS_SCORE=0 เป็นค่าเริ่มต้น ซึ่งหมายความว่าอัลกอริทึมของ Harris ถูกใช้เพื่อจัดอันดับฟีเจอร์ (คะแนนจะถูกเขียนลงใน KeyPoint::score และใช้เพื่อรักษาฟีเจอร์ nfeatures ที่ดีที่สุดไว้) FAST_SCORE=1 คือค่าทางเลือกของพารามิเตอร์ที่สร้างคีย์พอยต์ที่เสถียรน้อยกว่าเล็กน้อย แต่คำนวณได้เร็วกว่าเล็กน้อย
        'ORB/WTA_K': '2',               # จำนวนจุดที่สร้างแต่ละองค์ประกอบของตัวระบุ BRIEF ที่วางแนว ค่าเริ่มต้น 2 หมายถึง BRIEF ที่เราสุ่มคู่จุดและเปรียบเทียบความสว่างของจุดเหล่านั้น ดังนั้นเราจึงได้รับการตอบสนอง 0/1 ค่าที่เป็นไปได้อื่นๆ คือ 3 และ 4 ตัวอย่างเช่น 3 หมายถึงเราสุ่ม 3 จุด (แน่นอนว่าพิกัดจุดเหล่านี้เป็นแบบสุ่ม แต่สร้างขึ้นจากค่าซีดที่กำหนดไว้ล่วงหน้า ดังนั้นองค์ประกอบแต่ละองค์ประกอบของตัวระบุ BRIEF จึงถูกคำนวณจากสี่เหลี่ยมพิกเซลอย่างแม่นยำ) ค้นหาจุดที่มีความสว่างสูงสุดและดัชนีเอาต์พุตของผู้ชนะ (0, 1 หรือ 2) เอาต์พุตดังกล่าวจะใช้ 2 บิต ดังนั้นจึงต้องใช้รูปแบบพิเศษของระยะทางแฮมมิง ซึ่งแสดงเป็น NORM_HAMMING2 (2 บิตต่อช่อง) เมื่อ WTA_K=4 เราจะสุ่ม 4 จุดเพื่อคำนวณแต่ละช่อง (ซึ่งจะครอบครอง 2 บิตที่มีค่าที่เป็นไปได้ 0, 1, 2 หรือ 3)
        'KAZE/Threshold': '0.0002',
    }

    parameters_icp={
        'Icp/Strategy': '1',                    #ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/RangeMin': '0.0',                 # ระยะ Lidar ใกล้สุด
        'Icp/RangeMax': '40.0',                # ระยะ Lidar ไกลสุด
        'Icp/VoxelSize': '0.05',                   # ขนาดวอกเซลการสุ่มตัวอย่างแบบสม่ำเสมอ (0=ปิดใช้งาน)
        'Icp/PointToPlaneK': '100',              # จำนวนเพื่อนบ้านที่จะคำนวณค่าปกติสำหรับจุดต่อระนาบ
        'Icp/PointToPlaneRadius': '0',          # รัศมีเพื่อนบ้านที่จะคำนวณค่าปกติสำหรับจุดต่อระนาบ
        'Icp/PointToPlane': 'true',             # ใช้จุดต่อระนาบ ICP
        'Icp/Iterations': '30',                 # การวนซ้ำสูงสุด
        'Icp/Epsilon': '0.0001',                 # ตั้งค่า epsilon ของการแปลง (ค่าความแตกต่างสูงสุดที่อนุญาตระหว่างการแปลงสองครั้งติดต่อกัน) เพื่อให้พิจารณาว่าการเพิ่มประสิทธิภาพนั้นบรรจบกับโซลูชันสุดท้ายแล้ว
        'Icp/MaxTranslation': '0.5',            # การยอมรับการแก้ไขการแปล ICP สูงสุด (ม.)
        'Icp/MaxCorrespondenceDistance': '0.05', # ระยะทางสูงสุดสำหรับการโต้ตอบจุด
        'Icp/OutlierRatio': '0.6',              # อัตราส่วนที่ผิดปกติ
        'Icp/CorrespondenceRatio': '0.02',      # อัตราส่วนของการจับคู่ความสอดคล้องเพื่อยอมรับการแปลง
    }

    parameters_odom={
        'Odom/FilteringStrategy': '1',          # 0=No filtering 1=Kalman filtering 2=Particle filtering
        'Odom/KalmanMeasurementNoise': '0.001',  # ค่าความแปรปรวนร่วมของเสียงในกระบวนการ
        'Odom/KalmanProcessNoise': '0.0001',     # ค่าความแปรปรวนร่วมในการวัดกระบวนการ
        'Odom/Strategy': '0',                   # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F)
        'Odom/ResetCountdown': '1',             # รีเซ็ตโอโดมิเตอร์โดยอัตโนมัติหลังจากภาพติดต่อกัน X ภาพซึ่งโอโดมิเตอร์ไม่สามารถคำนวณได้ (ค่า = 0 ปิดใช้งานการรีเซ็ตอัตโนมัติ)
        'Odom/ValidDepthRatio': 'false',        # อัตราส่วนความลึกที่ถูกต้อง
        'Odom/Holonomic': 'true',               # หากหุ่นยนต์เป็นแบบโฮโลโนมิก (สามารถออกคำสั่งเคลื่อนที่ไปข้างหน้าได้) หากไม่เป็นเช่นนั้น ค่า y จะถูกประมาณจากค่า x และค่า yaw (y=x*tan(yaw)
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
            'odom_frame_id': 'odom',        # ไอดีเฟรมของ Odom
            # RTAB-Map's internal parameters are strings:
            'Odom/AlignWithGround': 'true',         # จัดตำแหน่งโอโดมิเตอร์ให้ตรงกับพื้นดินเมื่อเริ่มต้นใช้งาน  
        },parameters,parameters_rgbd,parameters_odom],
        remappings=[
            # ('odom', '/rgbd_odom')
        ],
        arguments=['--ros-args', '--log-level', 'fatal']
    )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[{
            'odom_frame_id': 'odom',        # ไอดีเฟรมของ Odom
            'expected_update_rate': 15.0,
            'deskewing': True,
            # RTAB-Map's internal parameters are strings:
            'Odom/AlignWithGround': 'false',
        },parameters,parameters_icp,parameters_odom],
        remappings=[
            ('odom', '/icp_odom')
        ]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        respawn=True,
        respawn_delay=1,
        parameters=[os.path.join(get_package_share_directory("rover_slam"), 'params', 'ekf.yaml')],
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
            'odom_frame_id': 'odom',        # ไอดีเฟรมของ Odom
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,          # ใช้งาน scan cloud Lidar
            'odom_sensor_sync': True,              # odom ของทุกเซนเซอร์ sync กัน
            # RTAB-Map's internal parameters are strings:
            'RGBD/ProximityMaxGraphDepth': '5',     # ความลึกสูงสุดจากตำแหน่งการปิดลูปปัจจุบัน/ครั้งสุดท้ายและสมมติฐานการปิดลูปในพื้นที่ ตั้งค่า 0 เพื่อละเว้น
            'RGBD/ProximityPathMaxNeighbors': '2',
            'RGBD/LinearUpdate': '0.01',            # การเคลื่อนที่เชิงเส้นขั้นต่ำเพื่ออัปเดตแผนที่ การซ้อมจะทำก่อนหน้านี้ ดังนั้นน้ำหนักจึงยังคงได้รับการอัปเดต
            'RGBD/AngularUpdate': '0.01',           # การเคลื่อนที่เชิงมุมขั้นต่ำเพื่ออัปเดตแผนที่ การซ้อมจะทำก่อนหน้านี้ ดังนั้นน้ำหนักจึงยังคงได้รับการอัปเดต
            'RGBD/CreateOccupancyGrid': 'true',     # สร้างตารางสำหรับ rgbd
            'RGBD/ForceOdom3DoF': 'true',           # odom แบบ 3D
            'RGBD/OptimizeFromGraphEnd': 'true',

            'Mem/IncrementalMemory': 'true',           # เปิดใช้งานหน่วยความจำแบบเพิ่มขึ้น
            'Mem/MapLabelsAdded': 'false',             # ไม่เพิ่มป้ายชื่อแผนที่
            'Mem/RawDescriptorsKept': 'false',        # ไม่เก็บข้อมูล descriptor
            'Mem/RecentWmRatio': '0.5',               # อัตราส่วนข้อมูลแผนที่ล่าสุด
            'Mem/ReduceGraph': 'true',                # ลดกราฟของแผนที่
            'Mem/LocalizationDataSaved': 'false',     # ไม่เก็บข้อมูลการหาตำแหน่ง
            'Mem/NotLinkedNodesKept': 'false',        # ไม่เก็บโหนดที่ไม่เชื่อมโยง
            'Mem/STMSize': '200',                     # ขนาดหน่วยความจำระยะสั้น
            'ImuFilter/ComplementaryBiasAlpha': '0.98',  # IMU complementary filter bias alpha
            'ImuFilter/ComplementaryDoAdpativeGain': 'true',
            'ImuFilter/MadgwickGain': '0.5',        # Madgwick filter gain
            'ImuFilter/MadgwickZeta': '0.1',        # Madgwick filter Zeta

            'Grid/MaxGroundAngle': '40.0',
            'Grid/MaxObstacleHeight': '2.0',        # เพิกเฉยสิ่งกีดขวางที่สูงเกิน (เมตร)
            'Grid/RangeMin': '0.0',
            'Grid/RangeMax': '40.0',
            'Grid/MinClusterSize': '60',
            'Grid/NormalK': '20',
            # 'Grid/FootprintHeight': '1.5',
            # 'Grid/FootprintLength': '0.59',
            # 'Grid/FootprintWidth': '0.51',
            'Grid/Sensor': '2',                     #Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
            'Grid/CellSize': '0.05',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',

            'GridGlobal/OccupancyThr': '0.60',

            'Mem/IncrementalMemory': mapping, # false is localization true is mapping
            'database_path': file,
        },parameters,parameters_rgbd,parameters_icp],
        remappings=[
            ('imu', '/D456_1/imu/data_filtered'),
            # ('gps/fix', 'gps/rawdata'),
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
        }]
    )

    lidar_tf = Node(
        name="lidar_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        respawn=True,
        respawn_delay=1,
        arguments=[
            "0.005", # translation_x
            "0.0", # translation_y
            "0.82", # translation_z
            "0.0",  # rotation_yaw
            "0.0",# rotation_pitch
            "0.0", # rotation_roll
            "base_link",  # parant D456_1_link
            "lidar_link",   # child
        ]
    )

    base_link_tf = Node(
        name="base_link_static_transform_publisher",
        package="tf2_ros",
        executable="static_transform_publisher",
        respawn=True,
        respawn_delay=1,
        arguments=[
            "0.32", # translation_x
            "0.0", # translation_y
            "0.45", # translation_z
            "0.0",  # rotation_yaw
            "0.4363",# rotation_pitch
            "0.0", # rotation_roll
            "base_link",  # parant
            "D456_1_link",   # child
        ]
    )

    base_footprint_tf = Node(
        name="base_footprint_static_transform_publisher",
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
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_mapping,
        declare_file,
        #s3_lidar,
        # realsense2_camera,
        lidar_tf,
        # base_footprint_tf,
        base_link_tf,
        rgbd_sync,
        rgbd_odometry,
        # icp_odometry,
        # ekf,
        rtabmap_slam,
        rtabmap_viz
    ])