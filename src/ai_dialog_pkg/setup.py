from setuptools import setup

package_name = 'ai_dialog_pkg'

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
    description='AI Dialog Node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_node = ai_dialog_pkg.ai_node:main',
        ],
    },
)