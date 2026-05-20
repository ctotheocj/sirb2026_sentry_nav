from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yaw_fusion',
            executable='yaw_fusion_node',
            name='yaw_fusion',
            output='screen',
            parameters=[
                {'calibration_timeout': 2.0},
                {'yaw_threshold': 0.01},
                {'use_tolerance': True},
                {'tolerance': 0.1},
                {'parent_frame': 'move_link'},
                {'child_frame': 'gimbal_link'},
            ]
        ),
    ])
