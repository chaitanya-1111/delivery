from setuptools import setup

package_name = 'speech_to_text_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'SpeechRecognition'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Speech to Text Node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = speech_to_text_pkg.stt_node:main',
        ],
    },
)