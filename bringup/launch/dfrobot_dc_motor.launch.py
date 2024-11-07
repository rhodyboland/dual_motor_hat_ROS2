# dfrobot_dc_motor.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

def generate_launch_description():
    # Launch configuration variables
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('dfrobot_dc_motor_hardware'), 'urdf', 'dfrobot_robot.urdf.xacro']
            ),
            # ' ',
            # 'use_sim_time:=', use_sim_time
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('dfrobot_dc_motor_hardware'),
            'config',
            'dfrobot_dc_motor.yaml',
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare('dfrobot_dc_motor_hardware'), 'rviz', 'dfrobot.rviz']
    # )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[('/diffbot_base_controller/cmd_vel', '/cmd_vel')],
        # arguments=['--ros-args', '--log-level', 'debug'], ('/diffbot_base_controller/odom', '/odom')
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    # )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # rviz_node,
    ]

    return LaunchDescription(nodes)
