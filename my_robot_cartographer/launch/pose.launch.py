import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=None,  # Brak pakietu, ponieważ używamy pełnej ścieżki do pliku wykonywalnego
            executable=os.path.join(os.getcwd(), 'src','my_robot_cartographer','my_robot_pose', 'my_robot_pose.py'),  # Ścieżka do pliku wykonywalnego
            name='my_robot_pose',
            output='screen',
            remappings=[('/amcl_pose', '/amcl_pose'), ('/odom', '/odom')],
        )
    ])
