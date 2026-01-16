import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    lidar_pkg_share = get_package_share_directory('sllidar_ros2')
    
    # Paths
    # xacro_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf.xacro')
    # ✅✅✅ 新增
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # Add Arduino Port Configuration here
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyUSB1') 
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')


    # 1. Robot Description (includes robot_state_publisher & joint_state_publisher)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'), 'launch', 'base.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # xacro_file = os.path.join(pkg_share, 'robot_description','urdf', 'base.urdf.xacro')

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': Command(['xacro ', xacro_file])
    #     }]
    # )

    # 3. Lidar Driver (A2M8)
    # ✅ 优化：调整激光雷达参数
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': lidar_frame_id,
            'inverted': False,
            'angle_compensate': True,
            'range_min': 0.25,         # ✅ 减小：允许检测更近的障碍物
            'range_max': 12.0,         # 最大检测距离
            'scan_frequency': 10.0,    # 扫描频率
        }],
        output='screen'
    )

    # ... (在 lidar_node 定义之后，base_driver 之前) ... 

    # 3.5 RealSense D435i Driver
    # Launch the camera with IMU and Depth enabled
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,       # 暂时关掉RGB节省带宽，除非你需要做视觉识别
            'rgb_camera.profile': '424x240x15', # ✅ 新增：降低RGB分辨率节省资源 (或者 640x480x15)
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': True,         # ✅ 开启陀螺仪
            'enable_accel': True,        # ✅ 开启加速度计
            'unite_imu_method': 1,       # ✅ 关键：0=None, 1=Copy, 2=Linear_Interpolation (推荐)
            'publish_tf': False,         # ❌ 关掉驱动自带的TF，我们要手动配置静态TF
            'depth_module.profile': '640x480x15', # 降低分辨率和帧率以减轻CPU负担
            # 'pointcloud.allow_no_texture_points': True,
            'gyro_fps': 200,             # IMU 频率
            'accel_fps': 100,
            'pointcloud.enable': True,   # ✅ 开启点云，Nav2避障需要
            'pointcloud.stream_filter': 2, # 下采样，减少点云数量
            'pointcloud.stream_index_filter': 0,

            
            # ✅ 移除 hole_filling 滤波器以消除障碍物残留
            # hole_filling 会将深度空洞用周围值填充，导致人走过后留下"拖影"
            'filters': 'spatial,decimation',
            'decimation_filter.filter_magnitude': 2,
        }],
        # ✅ 添加重映射：把合并后的话题名字统一一下，方便 EKF 听
        remappings=[
            ('/camera/camera/imu', '/camera/imu_combined') 
        ],
        output='screen'
    )

    # 3.6 Camera Static TF
    # ⚠️ 注意：你需要根据实际安装位置修改 x, y, z (米)
    # 假设摄像头安装在机器人中心前方 0.1米，高度 0.2米的位置
    # 参数顺序: x y z yaw pitch roll frame_id child_frame_id
    # camera_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_static_tf',
    #     arguments=['0.193', '0.0', '0.935', '0.0', '0.2618', '0.0', 'base_footprint', 'camera_link']
    # )

    # ... (在 camera_tf_node 之后)

    # ✅✅✅ 新增：深度图转激光雷达节点
    # 这会将 3D 深度图“压扁”成 2D 激光数据，专门用于 Costmap 清除障碍物
    # depth_to_scan_node = Node(
    #     package='depthimage_to_laserscan',
    #     executable='depthimage_to_laserscan_node',
    #     name='depthimage_to_laserscan',
    #     output='screen',
    #     parameters=[{
    #         'scan_height': 100,       # 关键参数：取中间 100 行像素进行投影（数值越大视野垂直覆盖越广）
    #         'range_min': 0.3,         # D435i 的最小盲区
    #         'range_max': 4.0,         # 超过这个距离被视为“空”（inf），用于清除障碍物
    #         'output_frame': 'camera_depth_frame' # 激光数据的坐标系
    #     }],
    #     remappings=[
    #         ('depth', '/camera/camera/depth/image_rect_raw'), # 订阅 RealSense 的深度图
    #         ('depth_camera_info', '/camera/camera/depth/camera_info'),
    #         ('scan', '/camera/scan')  # 发布伪激光话题
    #     ]
    # )

    # ... (记得在最后的 LaunchDescription 列表中加入 depth_to_scan_node)

    # ... 在 generate_launch_description 中 ...

    # ✅✅✅ 3D 点云转 2D 激光 (已禁用: STVL直接处理PointCloud2)
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     output='screen',
    #     parameters=[{
    #         'target_frame': 'base_footprint',
    #         'transform_tolerance': 0.2,
    #         'min_height': 0.03,
    #         'max_height': 2.0,
    #         'angle_min': -0.759,
    #         'angle_max': 0.759,
    #         'angle_increment': 0.0087,
    #         'scan_time': 0.067,
    #         'range_min': 0.30,
    #         'range_max': 3.0,
    #         'use_inf': True,
    #         'inf_epsilon': 1.0,
    #     }],
    #     remappings=[
    #         ('cloud_in', '/camera/depth/color/voxels'),
    #         ('scan', '/camera/scan')
    #     ]
    # )

    # ... 记得把 pointcloud_to_laserscan_node 加入到最后的 return LaunchDescription 列表中 ...

    # 🎯 点云体素化 (已禁用: STVL直接处理原始点云)
    # point_cloud_xyzrgb_node = Node(
    #     package='rtabmap_util',
    #     executable='point_cloud_xyzrgb',
    #     name='point_cloud_xyzrgb',
    #     output='screen',
    #     parameters=[{
    #         'decimation': 4,
    #         'voxel_size': 0.05,
    #         'approx_sync': True,
    #         'noise_filter_radius': 0.05,
    #         'noise_filter_min_neighbors': 5,
    #     }],
    #     remappings=[
    #         ('rgb/image', '/camera/camera/color/image_raw'),
    #         ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
    #         ('rgb/camera_info', '/camera/camera/color/camera_info'),
    #         ('cloud', '/camera/depth/color/voxels'),
    #     ]
    # )

    # 🎯 RTAB-Map 障碍物检测 (已禁用：改用更稳定的去噪+高度切割方案)
    # obstacles_detection_node = Node(...) 


    # ... (后面接 scan_fixer 和 base_driver) ... 

    # Scan Fixer (Fix Timestamp Lag)
    # Scan Fixer (Disabled for debugging ghosting issues)
    # scan_fixer = Node(
    #     package='my_robot_nav',
    #     executable='scan_fixer.py',
    #     name='scan_fixer',
    #     output='screen'
    # )

    # 4. Base Driver (Arduino Serial)
    base_driver = Node(
        package='my_robot_nav',
        executable='base_driver.py',
        name='base_driver',
        output='screen',
        parameters=[{
            'port': arduino_port,
            'baud': 115200
        }]
    )

    # 5. Kinematics (cmd_vel -> wheel_targets)
    # Subscribes: /cmd_vel
    # Publishes: /wheel_targets
    kinematics_node = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        output='screen'
    )

    # 6. Odometry (wheel_ticks -> odom + TF)
    # Subscribes: /wheel_ticks
    # Publishes: /odom, TF(odom->base_footprint)
    odometry_node = Node(
        package='my_robot_nav',
        executable='omni_odometry.py',
        name='wheel_odometry',
        parameters=[{
            'angle_scale_factor': LaunchConfiguration('angle_scale_factor'),
            'invert_z': LaunchConfiguration('invert_z'),
            'enc_sign_m1': LaunchConfiguration('enc_sign_m1'),
            'enc_sign_m2': LaunchConfiguration('enc_sign_m2'),
            'enc_sign_m3': LaunchConfiguration('enc_sign_m3'),
            'enc_sign_m4': LaunchConfiguration('enc_sign_m4'),
            'publish_tf': False  # ✅✅✅ 关键修改：关闭自带 TF
        }],
        output='screen'
    )

    # 7. Robot Localization (EKF)
    # Fuses wheel odometry (vx, vy) and IMU (v_yaw) to publish reliable odom->base_footprint TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odom_filtered')] # 可选，如果不重映射，默认发 /odometry/filtered
        # 注意：EKF 会发布 /tf (odom -> base_footprint)
    )

    # 5. SLAM & Navigation (Includes RViz)
    # This matches bringup.launch.py logic, but with use_sim_time=false
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (keep false for real robot)'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Lidar'),

        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for Arduino base controller'),

        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Lidar frame ID (must match URDF)'),

        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.52',
            description='Scale factor for odometry angle (increase if map updates too slow)'),

        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation for odometry'),

        DeclareLaunchArgument('enc_sign_m1', default_value='1', description='Encoder sign for Motor 1 (FL)'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1', description='Encoder sign for Motor 2 (FR)'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1', description='Encoder sign for Motor 3 (RR)'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1', description='Encoder sign for Motor 4 (RL)'),

        robot_description_launch,
        lidar_node,
        # scan_fixer,
        base_driver,
        kinematics_node,
        odometry_node,
        realsense_node,
        # camera_tf_node,
        # depth_to_scan_node,
        ekf_node,
        # pointcloud_to_laserscan_node,   # 已禁用: STVL直接处理PointCloud2
        # point_cloud_xyzrgb_node,        # 已禁用: STVL直接处理原始点云
        # obstacles_detection_node,
        
        # Delay SLAM/Nav slightly to ensure transforms are ready
        TimerAction(
            period=3.0,
            actions=[slam_nav_launch]
        )
    ])

