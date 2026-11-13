from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_line_follower',
            executable='line_follower',
            name='line_follower',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'cmd_vel_topic': '/cmd_vel',
                'debug_view': True,
                'roi_start_ratio': 0.5,
                'band_height': 120,
            }]
        )
    ])
