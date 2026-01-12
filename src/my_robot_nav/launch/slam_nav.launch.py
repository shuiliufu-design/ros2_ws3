import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_name = 'my_robot_nav'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Launch Configs
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox (Async mode for mapping)
    # Launch node directly to ensure config is loaded
    slam = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Nav2 Bringup (No map server if running SLAM, but SLAM toolbox provides map)
    # If SLAM is running, we don't need map_server to publish map, but we need other Nav2 nodes (planner, controller, bt_navigator, recoveries)
    # 'navigation_launch.py' brings up everything EXCEPT map_server and amcl usually.
    # Actually 'bringup_launch.py' brings up everything.
    # When doing SLAM, we commonly use 'navigation_launch.py' to avoid conflicting map server/AMCL.
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False' # Ensure independent nodes if issues arise
        }.items()
    )

    # RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'my_nav.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )


    # ✅ 修复：移除不必要的重映射
    # Nav2已经在nav2_params.yaml中配置了正确的cmd_vel话题链:
    # controller_server -> /cmd_vel_raw -> velocity_smoother -> /cmd_vel -> collision_monitor -> /cmd_vel_safe
    # 不需要额外的重映射，这会导致消息路由混乱
    nav2_remapped = GroupAction([
        nav2
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        slam,
        
        # Delay Nav2 to let SLAM start up
        TimerAction(
            period=3.0,
            actions=[nav2_remapped]
        ),
        
        rviz
    ])
