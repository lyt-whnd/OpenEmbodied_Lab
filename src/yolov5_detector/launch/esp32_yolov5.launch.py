#!/usr/bin/env python3
"""Launch ESP32-CAM, YOLOv5n, RobotLink, and gimbal control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the integrated active-vision launch description."""
    vision_share = get_package_share_directory('vision_demo')
    detector_share = get_package_share_directory('yolov5_detector')

    vision_config = os.path.join(
        vision_share,
        'config',
        'esp32_wifi.yaml',
    )
    detector_config = os.path.join(
        detector_share,
        'config',
        'yolov5n.yaml',
    )

    target_class = LaunchConfiguration('target_class')
    device = LaunchConfiguration('device')
    model_path = LaunchConfiguration('model_path')
    repo_path = LaunchConfiguration('repo_path')

    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='person'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument(
            'model_path',
            default_value=(
                '/home/lyt/robot_ws/src/yolov5_detector/'
                'weights/yolov5n.pt'
            ),
        ),
        DeclareLaunchArgument(
            'repo_path',
            default_value=(
                '/home/lyt/robot_ws/src/yolov5_detector/'
                'vendor/yolov5'
            ),
        ),
        Node(
            package='vision_demo',
            executable='esp32_camera',
            name='esp32_camera_node',
            output='screen',
            parameters=[vision_config],
        ),
        Node(
            package='yolov5_detector',
            executable='yolov5_detector',
            name='yolov5_detector_node',
            output='screen',
            parameters=[
                detector_config,
                {
                    'target_class': target_class,
                    'device': device,
                    'model_path': model_path,
                    'repo_path': repo_path,
                },
            ],
        ),
        Node(
            package='vision_demo',
            executable='robot_link',
            name='robot_link_node',
            output='screen',
            parameters=[vision_config],
        ),
        Node(
            package='vision_demo',
            executable='sensor_bridge',
            name='sensor_bridge_node',
            output='screen',
        ),
        Node(
            package='vision_demo',
            executable='gimbal_pd_websocket',
            name='gimbal_pd_websocket_node',
            output='screen',
            parameters=[vision_config],
        ),
    ])
