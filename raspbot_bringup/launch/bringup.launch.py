from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    enable_motors = LaunchConfiguration('enable_motors')
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic')
    enable_gpio_sensors = LaunchConfiguration('enable_gpio_sensors')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_gimbal = LaunchConfiguration('enable_gimbal')
    enable_oled = LaunchConfiguration('enable_oled')
    play_startup_sound = LaunchConfiguration('play_startup_sound')
    params_file = LaunchConfiguration('params_file')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('raspbot_hw'),
        'config',
        'raspbot_hw.yaml',
    ])

    hw_launch = PathJoinSubstitution([
        FindPackageShare('raspbot_hw'),
        'launch',
        'hw.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('enable_motors', default_value='true'),
        DeclareLaunchArgument('enable_ultrasonic', default_value='true'),
        DeclareLaunchArgument('enable_gpio_sensors', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_gimbal', default_value='true'),
        DeclareLaunchArgument('enable_oled', default_value='true'),
        DeclareLaunchArgument('play_startup_sound', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_launch),
            launch_arguments={
                'enable_motors': enable_motors,
                'enable_ultrasonic': enable_ultrasonic,
                'enable_gpio_sensors': enable_gpio_sensors,
                'enable_camera': enable_camera,
                'enable_gimbal': enable_gimbal,
                'enable_oled': enable_oled,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='raspbot_bringup',
            executable='startup_sound',
            name='startup_sound',
            output='screen',
            parameters=[
                params_file,
                {
                    'bringup_params_file': params_file,
                    'enable_motors': enable_motors,
                    'enable_ultrasonic': enable_ultrasonic,
                    'enable_gpio_sensors': enable_gpio_sensors,
                    'enable_camera': enable_camera,
                    'enable_gimbal': enable_gimbal,
                },
            ],
            condition=IfCondition(play_startup_sound),
        ),
    ])
