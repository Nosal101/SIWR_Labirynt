from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_control = Node(
        package="robot_control",
        executable="auto_control.py",
        name='auto_control',
        remappings=[
            ("cmd_vel", "cmd_vel"),
            ("scan", "scan"),
            ("odom", "odom")
        ],
    )
    ld.add_action(robot_control)
    return ld