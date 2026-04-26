import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitanya',
    maintainer_email='chaitanya@todo.todo',
    description='RPLidar bringup, diagnostics, TF, and watchdog package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_diagnostics_node = robot_lidar_pkg.lidar_diagnostics_node:main',
            'lidar_tf_node = robot_lidar_pkg.lidar_tf_node:main',
            'scan_watchdog_node = robot_lidar_pkg.scan_watchdog_node:main',
        ],
    },
)
