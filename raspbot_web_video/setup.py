from setuptools import setup

package_name = 'raspbot_web_video'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_video.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Minimal MJPEG web streamer for ROS 2 camera compressed images.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_video = raspbot_web_video.web_video_server:main',
            'face_recognition = raspbot_web_video.face_recognition_node:main',
        ],
    },
)
