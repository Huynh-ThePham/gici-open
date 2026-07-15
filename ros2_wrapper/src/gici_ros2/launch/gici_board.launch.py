"""Standard ROS 2 launch for GICI board RTK/IMU/Camera RRR.

Usage:
  ros2 launch gici_ros2 gici_board.launch.py \\
    config:=/path/to/rendered.yaml

For live topic-driven fusion use gici_live.launch.py instead (postfile configs
use batch replay and are not live).

See scripts/ros2/launch_gici_board_std.sh for full postfile/bag workflows.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            description='Absolute path to resolved GICI YAML config',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock when replaying bags with --clock',
        ),
        Node(
            package='gici_ros2',
            executable='gici_ros2_main',
            name='gici',
            output='screen',
            arguments=[config],
            parameters=[{
                'config_file': config,
                'use_sim_time': use_sim_time,
            }],
            sigterm_timeout='20',
            sigkill_timeout='5',
        ),
    ])
