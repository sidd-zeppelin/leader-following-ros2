#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            output='screen',
            parameters=[{
                'image_is_rectified': True,
                'marker_size': 0.15,
                'marker_id': 42,
                'reference_frame': 'TB3_2/base_link',
                'camera_frame': 'TB3_2/camera_rgb_optical_frame',
                'marker_frame': 'aruco_marker_frame',
            }],
            remappings=[
                ('/image', '/TB3_2/camera/image_raw'),
                ('/camera_info', '/TB3_2/camera/camera_info'),
            ],
        ),
    ])
