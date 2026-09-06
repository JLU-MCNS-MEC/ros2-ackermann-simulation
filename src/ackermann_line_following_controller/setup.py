from setuptools import setup
from glob import glob
import os


package_name = 'ackermann_line_following_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Camera line follower and Ackermann-to-Twist adapter.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = ackermann_line_following_controller.line_follower_node:main',
            'ackermann_to_twist = ackermann_line_following_controller.ackermann_to_twist_node:main',
            'waypoint_tracker = ackermann_line_following_controller.waypoint_tracker_node:main',
        ],
    },
)
