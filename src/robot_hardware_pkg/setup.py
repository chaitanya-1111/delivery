from setuptools import setup
import os
from glob import glob

package_name = 'robot_hardware_pkg'

setup(
    name=package_name,
    version='1.0.0',
    # The node modules currently live at the package root
    # (`hardware_interface_node.py`, `mock_arduino.py`) rather than under a
    # `robot_hardware_pkg/` Python package directory.
    # Use `py_modules` so ROS2 can import the entry points correctly.
    packages=[],
    py_modules=[
        'hardware_interface_node',
        'mock_arduino',
    ],
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
            'hardware_interface_node = hardware_interface_node:main',
            'mock_arduino = mock_arduino:main',
        ],
    },
)