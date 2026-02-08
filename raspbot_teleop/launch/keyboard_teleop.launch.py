from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Note: keyboard teleop requires an interactive TTY.
    return LaunchDescription(
        [
            Node(
                package="raspbot_teleop",
                executable="keyboard_teleop",
                name="keyboard_teleop",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
