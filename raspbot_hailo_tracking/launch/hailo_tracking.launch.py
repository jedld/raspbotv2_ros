from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hef_path = LaunchConfiguration('hef_path')
    model_name = LaunchConfiguration('model_name')
    labels_path = LaunchConfiguration('labels_path')
    input_topic = LaunchConfiguration('input_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    tracking_enable_topic = LaunchConfiguration('tracking_enable_topic')
    tracking_config_topic = LaunchConfiguration('tracking_config_topic')
    gimbal_topic = LaunchConfiguration('gimbal_topic')

    score_threshold = LaunchConfiguration('score_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    inference_fps = LaunchConfiguration('inference_fps')

    tracking_enabled = LaunchConfiguration('tracking_enabled')
    kp_pan_deg = LaunchConfiguration('kp_pan_deg')
    kp_tilt_deg = LaunchConfiguration('kp_tilt_deg')
    max_step_deg = LaunchConfiguration('max_step_deg')
    deadband_norm = LaunchConfiguration('deadband_norm')
    pan_sign = LaunchConfiguration('pan_sign')
    tilt_sign = LaunchConfiguration('tilt_sign')

    return LaunchDescription([
        DeclareLaunchArgument('hef_path', default_value=''),
        DeclareLaunchArgument('model_name', default_value=''),
        DeclareLaunchArgument('labels_path', default_value=''),
        DeclareLaunchArgument('input_topic', default_value='image_raw/compressed'),
        DeclareLaunchArgument('detections_topic', default_value='detections/json'),
        DeclareLaunchArgument('tracking_enable_topic', default_value='tracking/enable'),
        DeclareLaunchArgument('tracking_config_topic', default_value='tracking/config'),
        DeclareLaunchArgument('gimbal_topic', default_value='camera_gimbal/command_deg'),

        DeclareLaunchArgument('score_threshold', default_value='0.45'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('inference_fps', default_value='10.0'),

        DeclareLaunchArgument('tracking_enabled', default_value='false'),
        DeclareLaunchArgument('kp_pan_deg', default_value='28.0'),
        DeclareLaunchArgument('kp_tilt_deg', default_value='20.0'),
        DeclareLaunchArgument('max_step_deg', default_value='6.0'),
        DeclareLaunchArgument('deadband_norm', default_value='0.05'),
        DeclareLaunchArgument('pan_sign', default_value='-1'),
        DeclareLaunchArgument('tilt_sign', default_value='-1'),

        Node(
            package='raspbot_hailo_tracking',
            executable='hailo_detector',
            name='hailo_detector',
            output='screen',
            additional_env={'HAILO_MONITOR': '1'},
            parameters=[{
                'hef_path': hef_path,
                'model_name': model_name,
                'labels_path': labels_path,
                'input_topic': input_topic,
                'detections_topic': detections_topic,
                'tracking_enable_topic': tracking_enable_topic,
                'tracking_config_topic': tracking_config_topic,
                'gimbal_topic': gimbal_topic,
                'score_threshold': score_threshold,
                'iou_threshold': iou_threshold,
                'inference_fps': inference_fps,

                'tracking_enabled': tracking_enabled,
                'kp_pan_deg': kp_pan_deg,
                'kp_tilt_deg': kp_tilt_deg,
                'max_step_deg': max_step_deg,
                'deadband_norm': deadband_norm,
                'pan_sign': pan_sign,
                'tilt_sign': tilt_sign,
            }],
        ),
    ])
