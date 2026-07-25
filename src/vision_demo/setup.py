"""Install the vision_demo ROS 2 Python package."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'vision_demo'


setup(
    name=package_name,
    version='0.1.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/'
            'resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='lyt',
    maintainer_email='lyt@todo.todo',

    description=(
        'ROS 2 active vision and gimbal '
        'control demo with USB or ESP32-CAM'
    ),

    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            (
                'image_viewer = '
                'vision_demo.image_viewer_node:main'
            ),
            (
                'color_tracker = '
                'vision_demo.color_tracker_node:main'
            ),
            (
                'gimbal_controller = '
                'vision_demo.'
                'gimbal_controller_node:main'
            ),
            (
                'gimbal_pd_serial = '
                'vision_demo.'
                'gimbal_pd_serial_node:main'
            ),

            # ESP32 Wi-Fi 版本
            (
                'esp32_camera = '
                'vision_demo.'
                'esp32_camera_node:main'
            ),
            (
                'gimbal_pd_websocket = '
                'vision_demo.'
                'gimbal_pd_websocket_node:main'
            ),
            (
                'esp32_discovery = '
                'vision_demo.'
                'esp32_discovery:main'
            ),
            (
                'keyboard_motion = '
                'vision_demo.'
                'keyboard_motion_node:main'
            ),
        ],
    },
)
