from setuptools import setup

package_name = 'tts_player_pkg'

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
    description='Text to Speech Player',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = tts_player_pkg.tts_node:main',
        ],
    },
)
