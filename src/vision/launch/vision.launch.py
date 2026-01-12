from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Realsense camera launch (assuming widely used realsense2_camera package)
    # We try to launch it if not already running.
    # Note: If the user already has it running, this might conflict.
    # Ideally, we provide a flag to toggle camera launch.
    
    # Let's assume standard realsense2_camera launch usage
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',  # Essential for depth alignment
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'pointcloud.enable': 'false',
        }.items()
    )

    # Vision node
    vision_node = Node(
        package='vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'yolo_model_path': 'yolo11n.pt',
            'debug_mode': False,
        }]
    )

    return LaunchDescription([
        # realsense_launch, # Commented out by default to avoid conflict if user launched it, 
        # or we inform user to launch it. 
        # Actually, let's include it but maybe give an arg?
        # User said "how to run", usually implies "run everything".
        # But commonly camera is separate. 
        # Let's just create a vision launch that ONLY runs vision for now, 
        # BUT I will add a commented out section for camera.
        
        vision_node
    ])
