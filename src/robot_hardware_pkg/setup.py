from setuptools import setup
import os
from glob import glob

package_name = 'robot_hardware_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        'robot_hardware_pkg',
    ],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Robot Hardware Interface — Motor Control + Odometry',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'hardware_interface_node = robot_hardware_pkg.hardware_interface_node:main',
            'mock_arduino = robot_hardware_pkg.mock_arduino:main',
        ],
    },
)