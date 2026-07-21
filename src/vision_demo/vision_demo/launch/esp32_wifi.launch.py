#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_directory = (
        get_package_share_directory(
            'vision_demo'
        )
    )

    config_file = os.path.join(
        package_directory,
        'config',
        'esp32_wifi.yaml',
    )

    esp32_camera_node = Node(
        package='vision_demo',
        executable='esp32_camera',
        name='esp32_camera_node',
        output='screen',
        parameters=[config_file],
    )

    color_tracker_node = Node(
        package='vision_demo',
        executable='color_tracker',
        name='color_tracker_node',
        output='screen',
        parameters=[config_file],
    )

    websocket_control_node = Node(
        package='vision_demo',
        executable='gimbal_pd_websocket',
        name='gimbal_pd_websocket_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        esp32_camera_node,
        color_tracker_node,
        websocket_control_node,
    ])