import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dfrobot_dc_motor_hardware'),
        'config',
        'dfrobot_dc_motor.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['velocity_controllers'],
            output='screen'
        ),
    ])
