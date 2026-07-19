from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="velocity_demo",
            executable="velocity_publisher",
            name="velocity_publisher"
        ),
        Node(
            package="velocity_demo",
            executable="velocity_subscriber",
            name="velocity_subscriber"
        )
    ])