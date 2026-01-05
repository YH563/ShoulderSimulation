#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('shoulder')
    urdf_file_path = os.path.join(pkg_dir, 'config', 'urdf', 'shoulder.urdf') 
    rviz_config_path = os.path.join(pkg_dir, 'config', 'rviz', 'shoulder.rviz')  

    urdf_publisher_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',  
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file_path).read()  
        }],
        output='screen' 
    )

    rviz_node = Node(
        package='rviz2',  
        executable='rviz2', 
        name='rviz2',
        arguments=['-d', rviz_config_path],  
        output='screen'
    )

    # 打开 Rviz 窗口并加载配置文件，展示 URDF 模型
    return LaunchDescription([
        urdf_publisher_node,  # 先启动 URDF 发布
        rviz_node             # 再启动 Rviz 并加载配置
    ])