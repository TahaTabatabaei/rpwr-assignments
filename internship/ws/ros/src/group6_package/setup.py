from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'group6_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tahaos',
    maintainer_email='tabatabaei@uni-bremen.de',
    description='A package for the final project of the course RPWR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = group6_package.my_node:main',
            'keep_lane_with_teleop = group6_package.keep_lane_with_teleop:main',
            'knock_node = group6_package.knock_node:main',
            'keep_lane_real_robot = group6_package.keep_lane_real_robot:main',
            'keep_lane_with_teleop_filteredScan = group6_package.keep_lane_with_teleop_filteredScan:main'
            'fake_lidar_for_knock = group6_package.fake_lidar_for_knock:main',
            'robot_marker = group6_package.robot_marker:main',
            'advanced_knock_node = group6_package.advanced_knock_node:main',
            'knock_node_gazebo = group6_package.knock_node_gazebo:main',
        ],
    },
)
