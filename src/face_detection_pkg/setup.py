import os
from glob import glob
from setuptools import setup

package_name = 'face_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('face_detection_pkg/models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Face detection',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detector_node = face_detection_pkg.face_detector_node:main',
        ],
    },
)
