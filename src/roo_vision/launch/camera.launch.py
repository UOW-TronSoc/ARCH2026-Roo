from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='roo_camera',
            namespace='roo',
            parameters=[{
                'video_device': '/dev/roo_camera',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'io_method': 'mmap',
            }]
        )
    ])
