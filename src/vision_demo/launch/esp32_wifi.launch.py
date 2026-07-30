#!/usr/bin/env python3
"""Launch the ESP32 camera, tracker, RobotLink, and controller nodes."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """Build the ESP32 Wi-Fi control launch description."""
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

    robot_link_node = Node(
        package='vision_demo',
        executable='robot_link',
        name='robot_link_node',
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

    sensor_bridge_node = Node(
        package='vision_demo',
        executable='sensor_bridge',
        name='sensor_bridge_node',
        output='screen',
    )

    return LaunchDescription([
        esp32_camera_node,
        color_tracker_node,
        robot_link_node,
        sensor_bridge_node,
        websocket_control_node,
    ])
