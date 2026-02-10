from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hef_path = LaunchConfiguration('hef_path')
    model_name = LaunchConfiguration('model_name')
    input_topic = LaunchConfiguration('input_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_colorized_topic = LaunchConfiguration('depth_colorized_topic')
    enable_topic = LaunchConfiguration('enable_topic')
    inference_fps = LaunchConfiguration('inference_fps')
    colormap = LaunchConfiguration('colormap')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    max_depth_m = LaunchConfiguration('max_depth_m')

    return LaunchDescription([
        DeclareLaunchArgument('hef_path', default_value=''),
        DeclareLaunchArgument('model_name', default_value=''),
        DeclareLaunchArgument('input_topic', default_value='front_camera/compressed'),
        DeclareLaunchArgument('depth_image_topic', default_value='depth/image'),
        DeclareLaunchArgument('depth_colorized_topic', default_value='depth/colorized/compressed'),
        DeclareLaunchArgument('enable_topic', default_value='depth/enable'),
        DeclareLaunchArgument('inference_fps', default_value='5.0'),
        DeclareLaunchArgument('colormap', default_value='20'),       # COLORMAP_TURBO
        DeclareLaunchArgument('jpeg_quality', default_value='75'),
        DeclareLaunchArgument('max_depth_m', default_value='10.0'),  # NYU Depth V2 range

        Node(
            package='raspbot_hailo_tracking',
            executable='hailo_depth',
            name='hailo_depth',
            output='screen',
            parameters=[{
                'hef_path': hef_path,
                'model_name': model_name,
                'input_topic': input_topic,
                'depth_image_topic': depth_image_topic,
                'depth_colorized_topic': depth_colorized_topic,
                'enable_topic': enable_topic,
                'inference_fps': inference_fps,
                'colormap': colormap,
                'jpeg_quality': jpeg_quality,
                'max_depth_m': max_depth_m,
            }],
        ),
    ])
