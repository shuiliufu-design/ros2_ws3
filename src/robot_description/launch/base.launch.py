#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get package share dir
    pkg_robot_description = get_package_share_directory('robot_description')

    # 👉 Path to YOUR xacro file
    # If your file is in urdf/main/base.urdf.xacro, change to:
    # os.path.join(pkg_robot_description, 'urdf', 'main', 'base.urdf.xacro')
    default_model_path = os.path.join(
        pkg_robot_description,
        'urdf',
        'main',
        'base.urdf.xacro'
    )

    # Launch args
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Run xacro → URDF
    robot_description = Command(['xacro ', model])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        declare_model_arg,
        declare_use_sim_time_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])
