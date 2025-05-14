from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            namespace='camera',
            executable='realsense2_camera_node',

            parameters=[{
                'enable_color': True,
                'enable_depth': False,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30
            }]

        )
    ])