import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    # xacro_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf.xacro')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyUSB1') 
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='lidar_link')


    # 1. Robot Description (includes robot_state_publisher & joint_state_publisher)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'), 'launch', 'base.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Lidar Driver (A2M8)
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
            'angle_compensate': True
        }],
        remappings=[('/scan', '/scan_raw')],
        output='screen'
    )

    # # Scan Fixer
    # scan_fixer = Node(
    #     package='my_robot_nav',
    #     executable='scan_fixer.py',
    #     name='scan_fixer',
    #     output='screen'
    # )

    # 4. Base Driver (Arduino)
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

    # 5. Kinematics
    kinematics_node = Node(
        package='my_robot_nav',
        executable='cmd_vel_to_targets.py',
        name='cmdvel_to_targets',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'targets_topic': '/wheel_targets'
        }],
        output='screen'
    )

    # 6. Odometry (Target for Calibration)
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
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for Lidar'),

        DeclareLaunchArgument(
            'arduino_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Arduino'),

        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='lidar_link',
            description='Lidar frame ID'),

        DeclareLaunchArgument(
            'angle_scale_factor',
            default_value='1.52',
            description='Scale factor for odometry angle calibration'),

        DeclareLaunchArgument(
            'invert_z',
            default_value='False',
            description='Invert z-axis rotation'),

        DeclareLaunchArgument('enc_sign_m1', default_value='1'),
        DeclareLaunchArgument('enc_sign_m2', default_value='1'),
        DeclareLaunchArgument('enc_sign_m3', default_value='1'),
        DeclareLaunchArgument('enc_sign_m4', default_value='1'),

        robot_description_launch,
        lidar_node,
        # scan_fixer,
        base_driver,
        kinematics_node,
        odometry_node
    ])
