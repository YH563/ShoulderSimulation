#!/usr/bin/env python3
"""
同时启动Python和C++节点的Launch文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('shoulder')

    # Python节点
    python_node = Node(
        package='shoulder',
        executable='Plotter.py',  # 假设Python节点的可执行文件名为calculator.py
        name='Plotter',
        output='screen'
    )

    # C++节点
    cpp_node = Node(
        package='shoulder',
        executable='shoulder_core',  # 假设C++节点的可执行文件名为calculator_cpp_node
        name='shoulder_core',
        output='screen'
    )

    return LaunchDescription([
        python_node,
        cpp_node
    ])
