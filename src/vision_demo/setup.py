from setuptools import find_packages, setup

package_name = 'vision_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyt',
    maintainer_email='lyt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_viewer = vision_demo.image_viewer_node:main',
            'color_tracker = vision_demo.color_tracker_node:main',
            'gimbal_controller = vision_demo.gimbal_controller_node:main',
            'gimbal_pd_serial = vision_demo.gimbal_pd_serial_node:main',
        ],
    },
)
