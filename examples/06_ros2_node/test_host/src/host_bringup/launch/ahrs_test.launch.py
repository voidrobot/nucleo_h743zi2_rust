from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='host_bringup',
            executable='ahrs_verifier',
            name='ahrs_verifier_node',
            output='screen',
        )
    ])
