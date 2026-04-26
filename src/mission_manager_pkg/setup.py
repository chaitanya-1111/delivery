from setuptools import setup
import os
from glob import glob

package_name = 'mission_manager_pkg'

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
    description='The mission_manager_pkg package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = mission_manager_pkg.mission_manager_node:main',
            'order_queue_node = mission_manager_pkg.order_queue_node:main',
            'mission_logger_node = mission_manager_pkg.mission_logger_node:main',
        ],
    },
)
