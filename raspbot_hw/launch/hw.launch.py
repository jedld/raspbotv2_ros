import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    enable_bno055 = LaunchConfiguration('enable_bno055')
    enable_front_camera = LaunchConfiguration('enable_front_camera')
    enable_odometry = LaunchConfiguration('enable_odometry')
    enable_lidar = LaunchConfiguration('enable_lidar')
    params_file = LaunchConfiguration('params_file')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('raspbot_hw'),
        'config',
        'raspbot_hw.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('enable_motors', default_value='true'),
        DeclareLaunchArgument('enable_ultrasonic', default_value='true'),
        DeclareLaunchArgument('enable_gpio_sensors', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_gimbal', default_value='true'),
        DeclareLaunchArgument('enable_lightbar', default_value='true'),
        DeclareLaunchArgument('enable_oled', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_bno055', default_value='false'),
        DeclareLaunchArgument('enable_front_camera', default_value='false'),
        DeclareLaunchArgument('enable_odometry', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        Node(
            package='raspbot_hw',
            executable='motor_driver',
            name='motor_driver',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_motors),
        ),
        Node(
            package='raspbot_hw',
            executable='ultrasonic',
            name='ultrasonic',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_ultrasonic),
        ),
        Node(
            package='raspbot_hw',
            executable='gpio_sensors',
            name='gpio_sensors',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_gpio_sensors),
        ),
        Node(
            package='raspbot_hw',
            executable='opencv_camera',
            name='opencv_camera',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_camera),
        ),
        Node(
            package='raspbot_hw',
            executable='camera_gimbal',
            name='camera_gimbal',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_gimbal),
        ),

        Node(
            package='raspbot_hw',
            executable='lightbar',
            name='lightbar',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_lightbar),
        ),

        Node(
            package='raspbot_hw',
            executable='oled',
            name='oled',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_oled),
        ),

        Node(
            package='raspbot_hw',
            executable='imu_serial',
            name='imu_serial',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_imu),
        ),

        Node(
            package='raspbot_hw',
            executable='bno055_serial',
            name='bno055_serial',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_bno055),
        ),

        Node(
            package='raspbot_hw',
            executable='pi_camera',
            name='pi_camera',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_front_camera),
        ),

        Node(
            package='raspbot_hw',
            executable='odometry',
            name='odometry',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_odometry),
        ),

        # ── YDLidar T-mini Plus ──────────────────────────────────────
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'params', 'tmini_plus.yaml')],
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.05', '0', '0', '0', '1',
                       'base_link', 'laser_frame'],
            condition=IfCondition(enable_lidar),
        ),
    ])
