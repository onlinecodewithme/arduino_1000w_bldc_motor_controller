from setuptools import setup
import os
from glob import glob

package_name = 'arduino_motor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 bridge for communicating with Arduino motor controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arduino_motor_bridge.arduino_bridge_node:main',
            'test_arduino_commands = arduino_motor_bridge.test_arduino_commands:main',
        ],
    },
)
