from setuptools import setup
import os
from glob import glob

package_name = 'safety_supervisor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=['safety_supervisor_pkg'],
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
    description='Hardware-level safety supervisor with velocity gating and emergency stop',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor_node = safety_supervisor_pkg.safety_supervisor_node:main',
            'hardware_watchdog_node = safety_supervisor_pkg.hardware_watchdog_node:main',
            'safety_logger_node = safety_supervisor_pkg.safety_logger_node:main',
        ],
    },
)
