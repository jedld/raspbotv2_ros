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

    # Auto-follow
    follow_enabled = LaunchConfiguration('follow_enabled')
    track_class_id = LaunchConfiguration('track_class_id')
    track_label = LaunchConfiguration('track_label')
    follow_cmd_vel_topic = LaunchConfiguration('follow_cmd_vel_topic')
    follow_enable_topic = LaunchConfiguration('follow_enable_topic')
    follow_target_area_topic = LaunchConfiguration('follow_target_area_topic')
    follow_max_linear_topic = LaunchConfiguration('follow_max_linear_topic')
    follow_target_bbox_area = LaunchConfiguration('follow_target_bbox_area')
    follow_kp_linear = LaunchConfiguration('follow_kp_linear')
    follow_ki_linear = LaunchConfiguration('follow_ki_linear')
    follow_kd_linear = LaunchConfiguration('follow_kd_linear')
    follow_kp_angular = LaunchConfiguration('follow_kp_angular')
    follow_ki_angular = LaunchConfiguration('follow_ki_angular')
    follow_kd_angular = LaunchConfiguration('follow_kd_angular')
    follow_max_linear = LaunchConfiguration('follow_max_linear')
    follow_max_angular = LaunchConfiguration('follow_max_angular')
    follow_linear_deadband = LaunchConfiguration('follow_linear_deadband')
    follow_angular_deadband = LaunchConfiguration('follow_angular_deadband')
    follow_lost_timeout_sec = LaunchConfiguration('follow_lost_timeout_sec')
    follow_use_gimbal_feedback = LaunchConfiguration('follow_use_gimbal_feedback')
    follow_scan_speed_deg = LaunchConfiguration('follow_scan_speed_deg')
    follow_scan_pause_sec = LaunchConfiguration('follow_scan_pause_sec')
    follow_scan_max_sweeps = LaunchConfiguration('follow_scan_max_sweeps')
    follow_gimbal_recenter_kp = LaunchConfiguration('follow_gimbal_recenter_kp')
    follow_angular_blend = LaunchConfiguration('follow_angular_blend')
    follow_ultrasonic_topic = LaunchConfiguration('follow_ultrasonic_topic')
    follow_obstacle_stop_m = LaunchConfiguration('follow_obstacle_stop_m')
    follow_obstacle_slow_m = LaunchConfiguration('follow_obstacle_slow_m')

    # Depth estimation
    depth_hef_path = LaunchConfiguration('depth_hef_path')
    depth_input_topic = LaunchConfiguration('depth_input_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_colorized_topic = LaunchConfiguration('depth_colorized_topic')
    depth_enable_topic = LaunchConfiguration('depth_enable_topic')
    depth_inference_fps = LaunchConfiguration('depth_inference_fps')

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

        # Auto-follow launch arguments
        DeclareLaunchArgument('track_class_id', default_value='-1'),
        DeclareLaunchArgument('track_label', default_value='face'),
        DeclareLaunchArgument('follow_enabled', default_value='false'),
        DeclareLaunchArgument('follow_cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('follow_enable_topic', default_value='follow/enable'),
        DeclareLaunchArgument('follow_target_area_topic', default_value='follow/target_area'),
        DeclareLaunchArgument('follow_max_linear_topic', default_value='follow/max_linear'),
        DeclareLaunchArgument('follow_target_bbox_area', default_value='0.04'),
        DeclareLaunchArgument('follow_kp_linear', default_value='0.8'),
        DeclareLaunchArgument('follow_ki_linear', default_value='0.05'),
        DeclareLaunchArgument('follow_kd_linear', default_value='0.1'),
        DeclareLaunchArgument('follow_kp_angular', default_value='1.2'),
        DeclareLaunchArgument('follow_ki_angular', default_value='0.05'),
        DeclareLaunchArgument('follow_kd_angular', default_value='0.1'),
        DeclareLaunchArgument('follow_max_linear', default_value='0.3'),
        DeclareLaunchArgument('follow_max_angular', default_value='0.8'),
        DeclareLaunchArgument('follow_linear_deadband', default_value='0.02'),
        DeclareLaunchArgument('follow_angular_deadband', default_value='0.05'),
        DeclareLaunchArgument('follow_lost_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('follow_use_gimbal_feedback', default_value='true'),
        DeclareLaunchArgument('follow_scan_speed_deg', default_value='30.0'),
        DeclareLaunchArgument('follow_scan_pause_sec', default_value='0.4'),
        DeclareLaunchArgument('follow_scan_max_sweeps', default_value='3'),
        DeclareLaunchArgument('follow_gimbal_recenter_kp', default_value='0.10'),
        DeclareLaunchArgument('follow_angular_blend', default_value='0.5'),
        DeclareLaunchArgument('follow_ultrasonic_topic', default_value='ultrasonic/range'),
        DeclareLaunchArgument('follow_obstacle_stop_m', default_value='0.20'),
        DeclareLaunchArgument('follow_obstacle_slow_m', default_value='0.50'),

        # Depth estimation arguments
        DeclareLaunchArgument('depth_hef_path', default_value=''),
        DeclareLaunchArgument('depth_input_topic', default_value='front_camera/compressed'),
        DeclareLaunchArgument('depth_image_topic', default_value='depth/image'),
        DeclareLaunchArgument('depth_colorized_topic', default_value='depth/colorized/compressed'),
        DeclareLaunchArgument('depth_enable_topic', default_value='depth/enable'),
        DeclareLaunchArgument('depth_inference_fps', default_value='5.0'),

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

                'follow_enabled': follow_enabled,
                'track_class_id': track_class_id,
                'track_label': track_label,
                'follow_cmd_vel_topic': follow_cmd_vel_topic,
                'follow_enable_topic': follow_enable_topic,
                'follow_target_area_topic': follow_target_area_topic,
                'follow_max_linear_topic': follow_max_linear_topic,
                'follow_target_bbox_area': follow_target_bbox_area,
                'follow_kp_linear': follow_kp_linear,
                'follow_ki_linear': follow_ki_linear,
                'follow_kd_linear': follow_kd_linear,
                'follow_kp_angular': follow_kp_angular,
                'follow_ki_angular': follow_ki_angular,
                'follow_kd_angular': follow_kd_angular,
                'follow_max_linear': follow_max_linear,
                'follow_max_angular': follow_max_angular,
                'follow_linear_deadband': follow_linear_deadband,
                'follow_angular_deadband': follow_angular_deadband,
                'follow_lost_timeout_sec': follow_lost_timeout_sec,
                'follow_use_gimbal_feedback': follow_use_gimbal_feedback,
                'follow_scan_speed_deg': follow_scan_speed_deg,
                'follow_scan_pause_sec': follow_scan_pause_sec,
                'follow_scan_max_sweeps': follow_scan_max_sweeps,
                'follow_gimbal_recenter_kp': follow_gimbal_recenter_kp,
                'follow_angular_blend': follow_angular_blend,
                'follow_ultrasonic_topic': follow_ultrasonic_topic,
                'follow_obstacle_stop_m': follow_obstacle_stop_m,
                'follow_obstacle_slow_m': follow_obstacle_slow_m,

                'depth_hef_path': depth_hef_path,
                'depth_input_topic': depth_input_topic,
                'depth_image_topic': depth_image_topic,
                'depth_colorized_topic': depth_colorized_topic,
                'depth_enable_topic': depth_enable_topic,
                'depth_inference_fps': depth_inference_fps,
            }],
        ),
    ])
