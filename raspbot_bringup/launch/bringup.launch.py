import os

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
    enable_lightbar = LaunchConfiguration('enable_lightbar')
    enable_oled = LaunchConfiguration('enable_oled')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_front_camera = LaunchConfiguration('enable_front_camera')
    play_startup_sound = LaunchConfiguration('play_startup_sound')
    enable_web_video = LaunchConfiguration('enable_web_video')
    enable_hailo = LaunchConfiguration('enable_hailo')
    hailo_hef_path = LaunchConfiguration('hailo_hef_path')
    hailo_labels_path = LaunchConfiguration('hailo_labels_path')
    hailo_pan_sign = LaunchConfiguration('hailo_pan_sign')
    hailo_tilt_sign = LaunchConfiguration('hailo_tilt_sign')
    tracking_config_topic = LaunchConfiguration('tracking_config_topic')
    follow_enable_topic = LaunchConfiguration('follow_enable_topic')
    depth_hef_path = LaunchConfiguration('depth_hef_path')
    enable_lidar = LaunchConfiguration('enable_lidar')
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

    web_video_launch = PathJoinSubstitution([
        FindPackageShare('raspbot_web_video'),
        'launch',
        'web_video.launch.py',
    ])

    hailo_tracking_launch = PathJoinSubstitution([
        FindPackageShare('raspbot_hailo_tracking'),
        'launch',
        'hailo_tracking.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('enable_motors', default_value='true'),
        DeclareLaunchArgument('enable_ultrasonic', default_value='true'),
        DeclareLaunchArgument('enable_gpio_sensors', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_gimbal', default_value='true'),
        DeclareLaunchArgument('enable_lightbar', default_value='true'),
        DeclareLaunchArgument('enable_oled', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_front_camera', default_value='true'),
        DeclareLaunchArgument('play_startup_sound', default_value='true'),
        DeclareLaunchArgument('enable_web_video', default_value='true'),
        DeclareLaunchArgument('enable_hailo', default_value='true'),
        DeclareLaunchArgument('hailo_hef_path', default_value=
            os.path.expanduser('~/.local/share/raspbot/models/hailo8/yolov5s_personface.hef')),
        DeclareLaunchArgument('hailo_labels_path', default_value=
            os.path.expanduser('~/.local/share/raspbot/models/hailo8/personface.labels')),
        DeclareLaunchArgument('hailo_pan_sign', default_value='-1'),
        DeclareLaunchArgument('hailo_tilt_sign', default_value='-1'),
        DeclareLaunchArgument('tracking_config_topic', default_value='tracking/config'),
        DeclareLaunchArgument('follow_enable_topic', default_value='follow/enable'),
        DeclareLaunchArgument('depth_hef_path', default_value=
            os.path.expanduser('~/.local/share/raspbot/models/hailo8/fast_depth.hef')),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_launch),
            launch_arguments={
                'enable_motors': enable_motors,
                'enable_ultrasonic': enable_ultrasonic,
                'enable_gpio_sensors': enable_gpio_sensors,
                'enable_camera': enable_camera,
                'enable_gimbal': enable_gimbal,
                'enable_lightbar': enable_lightbar,
                'enable_oled': enable_oled,
                'enable_imu': enable_imu,
                'enable_front_camera': enable_front_camera,
                'enable_lidar': enable_lidar,
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_video_launch),
            launch_arguments={
                'topic': 'image_raw/compressed',
                'detections_topic': 'detections/json',
                'tracking_enable_topic': 'tracking/enable',
                'tracking_config_topic': tracking_config_topic,
                'follow_enable_topic': follow_enable_topic,
            }.items(),
            condition=IfCondition(enable_web_video),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hailo_tracking_launch),
            launch_arguments={
                'hef_path': hailo_hef_path,
                'labels_path': hailo_labels_path,
                'input_topic': 'image_raw/compressed',
                'detections_topic': 'detections/json',
                'tracking_enable_topic': 'tracking/enable',
                'tracking_config_topic': tracking_config_topic,
                'follow_enable_topic': follow_enable_topic,
                'gimbal_topic': 'camera_gimbal/command_deg',
                'pan_sign': hailo_pan_sign,
                'tilt_sign': hailo_tilt_sign,
                'depth_hef_path': depth_hef_path,
            }.items(),
            condition=IfCondition(enable_hailo),
        ),
    ])
