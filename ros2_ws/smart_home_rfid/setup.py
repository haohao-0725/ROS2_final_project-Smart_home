import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'smart_home_rfid'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools', 'flask', 'flask-socketio', 'pyserial', 'eventlet'],
    zip_safe=True,
    maintainer='ros2vm',
    maintainer_email='ros2vm@todo.todo',
    description='RFID Smart Home System with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = smart_home_rfid.arduino_bridge:main',
        ],
    },
)