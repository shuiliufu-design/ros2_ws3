#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
返回原点脚本 - 使用 Nav2 Simple Commander API
机器人到达目标后，运行此脚本让机器人返回起始点 (0, 0)

使用方法:
    ros2 run my_robot_nav return_home.py
    或
    python3 ~/ros2_ws3/src/my_robot_nav/src/return_home.py
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math


def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    # 等待 Nav2 完全激活
    print('⏳ 等待 Nav2 导航栈激活...')
    navigator.waitUntilNav2Active()
    print('✅ Nav2 已激活!')
    
    # 创建返回原点的目标 Pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # 原点坐标 (0, 0)，朝向角度 theta = 0
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    
    # 四元数表示 yaw = 0 (朝向正X轴)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    
    print('🚀 开始返回原点 (0, 0)...')
    navigator.goToPose(goal_pose)
    
    # 等待导航完成，并显示实时反馈
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:  # 每5次循环打印一次
            distance = feedback.distance_remaining
            print(f'📍 距离原点: {distance:.2f} 米')
    
    # 获取导航结果
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print('✅ 成功返回原点!')
    elif result == TaskResult.CANCELED:
        print('⚠️ 导航被取消')
    elif result == TaskResult.FAILED:
        print('❌ 返回原点失败')
    else:
        print(f'⚠️ 未知结果: {result}')
    
    # 清理
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
