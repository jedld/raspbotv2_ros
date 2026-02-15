from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_name = LaunchConfiguration("device_name")
    device_address = LaunchConfiguration("device_address")
    follow_enable_topic = LaunchConfiguration("follow_enable_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="RaspbotCardputer"),
            DeclareLaunchArgument("device_address", default_value=""),
            DeclareLaunchArgument("follow_enable_topic", default_value="follow/enable"),
            Node(
                package="raspbot_teleop",
                executable="bt_cardputer_teleop",
                name="bt_cardputer_teleop",
                output="screen",
                parameters=[
                    {
                        "device_name": device_name,
                        "device_address": device_address,
                        "follow_enable_topic": follow_enable_topic,
                    }
                ],
            ),
        ]
    )
