from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topic = LaunchConfiguration('topic')
    bind = LaunchConfiguration('bind')
    port = LaunchConfiguration('port')
    fps_limit = LaunchConfiguration('fps_limit')
    gimbal_topic = LaunchConfiguration('gimbal_topic')
    pan_min_deg = LaunchConfiguration('pan_min_deg')
    pan_max_deg = LaunchConfiguration('pan_max_deg')
    tilt_min_deg = LaunchConfiguration('tilt_min_deg')
    tilt_max_deg = LaunchConfiguration('tilt_max_deg')
    pan_neutral_deg = LaunchConfiguration('pan_neutral_deg')
    tilt_neutral_deg = LaunchConfiguration('tilt_neutral_deg')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    max_angular_rps = LaunchConfiguration('max_angular_rps')
    cmd_timeout_sec = LaunchConfiguration('cmd_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='image_raw/compressed'),
        DeclareLaunchArgument('bind', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('fps_limit', default_value='15.0'),
        DeclareLaunchArgument('gimbal_topic', default_value='camera_gimbal/command_deg'),
        DeclareLaunchArgument('pan_min_deg', default_value='0.0'),
        DeclareLaunchArgument('pan_max_deg', default_value='180.0'),
        DeclareLaunchArgument('tilt_min_deg', default_value='0.0'),
        DeclareLaunchArgument('tilt_max_deg', default_value='110.0'),
        DeclareLaunchArgument('pan_neutral_deg', default_value='90.0'),
        DeclareLaunchArgument('tilt_neutral_deg', default_value='90.0'),

        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.25'),
        DeclareLaunchArgument('max_angular_rps', default_value='1.2'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),

        Node(
            package='raspbot_web_video',
            executable='web_video',
            name='web_video',
            output='screen',
            parameters=[{
                'topic': topic,
                'bind': bind,
                'port': port,
                'fps_limit': fps_limit,
                'gimbal_topic': gimbal_topic,
                'pan_min_deg': pan_min_deg,
                'pan_max_deg': pan_max_deg,
                'tilt_min_deg': tilt_min_deg,
                'tilt_max_deg': tilt_max_deg,
                'pan_neutral_deg': pan_neutral_deg,
                'tilt_neutral_deg': tilt_neutral_deg,

                'cmd_vel_topic': cmd_vel_topic,
                'max_linear_mps': max_linear_mps,
                'max_angular_rps': max_angular_rps,
                'cmd_timeout_sec': cmd_timeout_sec,
            }],
        ),
    ])
