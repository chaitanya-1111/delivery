from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_client = robot_navigation_pkg.nav_client_node:main',
            'nav_status = robot_navigation_pkg.nav_status_node:main',
            'cmd_vel_arbiter = robot_navigation_pkg.cmd_vel_arbiter_node:main',
        ],
    },
)
