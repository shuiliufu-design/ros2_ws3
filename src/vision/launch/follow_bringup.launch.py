import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---------------- Arguments ----------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyUSB0') 
    
    # ---------------- Robot Parts ----------------
    # (Based on user provided code)
    pkg_name = 'my_robot_nav'
    pkg_share_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share_path, 'urdf', 'my_car.urdf.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port, 
            'serial_baudrate': 115200, 
            'frame_id': 'lidar_link',
            'inverted': False, 
            'angle_compensate': True
        }],
        remappings=[('/scan', '/scan_raw')],
        output='screen'
    )

    scan_fixer = Node(
        package='my_robot_nav',
        executable='scan_fixer.py',
        name='scan_fixer',
        output='screen'
    )

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

    kinematics_node = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        output='screen'
    )

    odometry_node = Node(
        package='my_robot_nav',
        executable='omni_odometry.py',
        name='wheel_odometry',
        parameters=[{
            # Using defaults or what was provided in prompt
            'angle_scale_factor': 1.04,
            'invert_z': False,
        }],
        output='screen'
    )

    # ---------------- Camera ----------------
    # Launch Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'pointcloud.enable': 'false',
        }.items()
    )

    # ---------------- Vision ----------------
    vision_node = Node(
        package='vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            # IMPORTANT: We want coordinates in ROBOT frame for following
            'global_frame': 'base_footprint', 
            'debug_mode': False,
        }]
    )

    # ---------------- Follower ----------------
    person_follower = Node(
        package='vision',
        executable='person_follower.py',
        name='person_follower',
        output='screen',
        parameters=[{
            'target_distance': 1.0, # meters
            'max_linear_x': 0.3, 
            'kp_angular': 1.5
        }]
    )

    return LaunchDescription([
        # Robot Chassis
        robot_state_publisher,
        joint_state_publisher,
        lidar_node,
        scan_fixer,
        base_driver,
        kinematics_node,
        odometry_node,

        # Sensors
        realsense_launch,

        # Logic
        TimerAction(
            period=5.0, # Wait for sensors to maintain proper TF
            actions=[vision_node]
        ),
        
        TimerAction(
            period=7.0,
            actions=[person_follower]
        )
    ])
