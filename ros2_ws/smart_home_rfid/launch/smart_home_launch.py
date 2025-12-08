#!/usr/bin/env python3
"""
ROS2 Launch file for Smart Home RFID System
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for Arduino serial communication'
    )

    whitelist_file_arg = DeclareLaunchArgument(
        'whitelist_file',
        default_value='/home/ros2vm/final_project_ros2/ros2_ws/smart_home_rfid/config/whitelist.json',
        description='Path to whitelist JSON file'
    )

    arduino_bridge_node = Node(
        package='smart_home_rfid',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }]
    )

    auth_node = Node(
        package='smart_home_rfid',
        executable='auth_node',
        name='auth_node',
        output='screen',
        parameters=[{
            'whitelist_file': LaunchConfiguration('whitelist_file'),
        }]
    )

    web_server_node = Node(
        package='smart_home_rfid',
        executable='web_server',
        name='web_server',
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        whitelist_file_arg,
        arduino_bridge_node,
        auth_node,
        web_server_node,
    ])