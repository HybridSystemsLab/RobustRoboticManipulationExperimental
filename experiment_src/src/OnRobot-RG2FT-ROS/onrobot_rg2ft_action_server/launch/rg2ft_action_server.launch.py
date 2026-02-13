from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onrobot_rg2ft_action_server',
            executable='onrobot_rg2ft_action_server',
            name='onrobot_rg2ft_action_server',
            output='screen',
            parameters=[{
                'action_server_name': 'gripper_controller/gripper_cmd',
                'min_angle': 0.0,
                'max_angle': 0.93,
                'min_effort': 3.0,
                'max_effort': 40.0,
                'default_effort': 10.0,
                'control_topic': 'gripper/ctrl',
                'state_topic': 'gripper/states',
                'joint_states_topic': 'joint_states',
                'joint_name': 'left_outer_knuckle_joint',
            }]
        )
    ])
