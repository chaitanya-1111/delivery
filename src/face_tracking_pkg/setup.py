from setuptools import setup
import os
from glob import glob

package_name = 'face_tracking_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Face tracking servo controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_tracker_node = face_tracking_pkg.face_tracker_node:main',
        ],
    },
)
