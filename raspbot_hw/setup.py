from setuptools import setup

package_name = 'raspbot_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/raspbot_hw.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Hardware drivers for Yahboom Raspbot V2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = raspbot_hw.motor_driver_node:main',
            'motor_id_test = raspbot_hw.motor_id_test_node:main',
            'ultrasonic = raspbot_hw.ultrasonic_node:main',
            'gpio_sensors = raspbot_hw.gpio_sensors_node:main',
            'opencv_camera = raspbot_hw.opencv_camera_node:main',
            'camera_gimbal = raspbot_hw.camera_gimbal_node:main',
            'oled = raspbot_hw.oled_node:main',
        ],
    },
)
