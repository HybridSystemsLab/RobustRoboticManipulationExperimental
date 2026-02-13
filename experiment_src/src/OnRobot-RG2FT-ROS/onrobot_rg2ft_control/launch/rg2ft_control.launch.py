from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onrobot_rg2ft_control',
            executable='rg2ft_driver',
            name='onrobot_rg2ft_driver',
            output='screen',
            parameters=[{
                'ip': '192.168.1.1',
                'port': '502',
            }]
        )
    ])
