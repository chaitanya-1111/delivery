from setuptools import setup
import os
from glob import glob

package_name = 'robot_slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitanya',
    maintainer_email='chaitanya@todo.todo',
    description='SLAM and mapping for the robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_session = robot_slam_pkg.scripts.mapping_session:main',
            'dynamic_obstacle_filter_node = robot_slam_pkg.dynamic_obstacle_filter_node:main',
            'map_quality_node = robot_slam_pkg.map_quality_node:main',
            'map_manager_node = robot_slam_pkg.map_manager_node:main',
        ],
    },
)
