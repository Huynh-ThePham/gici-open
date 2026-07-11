"""Launch the GICI ROS 2 node for UrbanNav.

Usage:
  ros2 launch gici_ros2 urbannav.launch.py config:=/abs/path/to/ros_urbannav.yaml

The config file must already have its OUTPUT_DIR placeholder resolved (see
scripts/ros2/run_urbannav_ros2.sh, which prepares a runtime config and then
plays the ROS 2 bag alongside this node).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            description='Absolute path to the resolved ros_urbannav.yaml config file',
        ),
        Node(
            package='gici_ros2',
            executable='gici_ros2_main',
            name='gici',
            output='screen',
            arguments=[config],
        ),
    ])
