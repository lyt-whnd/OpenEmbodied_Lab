"""Install the yolov5_detector ROS 2 Python package."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'yolov5_detector'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyt',
    maintainer_email='lyt@todo.todo',
    description=(
        'PyTorch YOLOv5n detector integrated with the existing '
        'ESP32-CAM and gimbal-control ROS 2 nodes.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            (
                'yolov5_detector = '
                'yolov5_detector.detector_node:main'
            ),
        ],
    },
)
