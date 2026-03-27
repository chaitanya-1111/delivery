from setuptools import setup

package_name = 'session_manager_pkg'

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
    description='Session State Machine',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'session_node = session_manager_pkg.session_node:main',
        ],
    },
)