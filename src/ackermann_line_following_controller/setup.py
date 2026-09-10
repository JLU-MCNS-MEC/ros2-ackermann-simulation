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
            'rgbd_audit = ackermann_line_following_controller.rgbd_audit:main',
            (
                'visual_dataset_recorder = '
                'ackermann_line_following_controller.visual_dataset_recorder:main'
            ),
            (
                'visual_policy_train = '
                'ackermann_line_following_controller.visual_policy_train:main'
            ),
            (
                'visual_policy = '
                'ackermann_line_following_controller.visual_policy_node:main'
            ),
            'navigation_regression = ackermann_line_following_controller.navigation_regression:main',
            'semantic_navigation = ackermann_line_following_controller.semantic_navigation_node:main',
            'line_follower = ackermann_line_following_controller.line_follower_node:main',
            'ackermann_to_twist = ackermann_line_following_controller.ackermann_to_twist_node:main',
            'twist_to_ackermann = ackermann_line_following_controller.twist_to_ackermann_node:main',
            'waypoint_tracker = ackermann_line_following_controller.waypoint_tracker_node:main',
            'nav2_waypoint_sender = ackermann_line_following_controller.nav2_waypoint_sender:main',
            'ackermann_dynamics_test = ackermann_line_following_controller.dynamics_test_node:main',
            'navigation_diagnostics = ackermann_line_following_controller.navigation_diagnostics_node:main',
            'navigation_plotter = ackermann_line_following_controller.navigation_plotter:main',
            'navigation_experiment = ackermann_line_following_controller.navigation_experiment_node:main',
        ],
    },
)
