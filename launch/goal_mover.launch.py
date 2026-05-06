from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_mover',
            executable='goal_mover_node',
            name='goal_mover_node',
            output='screen',
            remappings=[
                ('/odom',    '/robot/odom'),
                ('/cmd_vel', '/robot/cmd_vel'),
            ],
            parameters=[{
                'desired_linear_vel':      0.2,
                'linear_accel':            0.4,
                'linear_decel':            0.6,
                'angular_rate_deg_per_cm': 1.0,
                'angular_deadband':        0.02,
                'xy_goal_tolerance':       0.05,
                'yaw_goal_tolerance':      0.05,
                'control_rate':           20.0,
            }],
        ),
    ])
