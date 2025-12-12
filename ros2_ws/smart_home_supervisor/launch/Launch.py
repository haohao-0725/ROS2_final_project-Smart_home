from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    supervisor_node = Node(
        package='smart_home_supervisor',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[
            # 你可以在這裡覆寫參數（不寫也可以，會用 node 內 declare_parameter 預設值）
            # {'temp_user_timeout_sec': 600},
            # {'status_period_sec': 1.0},
        ],
    )

    supervisor_gui = Node(
        package='smart_home_supervisor',
        executable='supervisor_gui',
        name='supervisor_gui',
        output='screen',
    )

    return LaunchDescription([
        supervisor_node,
        supervisor_gui,
    ])
