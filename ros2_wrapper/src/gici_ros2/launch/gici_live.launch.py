"""Launch GICI as a live ROS 2 fusion node (topic-driven, not post-file batch).

The estimator runs on MultiSensorEstimating when replay.enable is false and all
sensor streamers are type: ros. rclcpp::spin processes subscription callbacks;
fusion threads consume measurements in real time.

Usage:
  # Board live (connect sensors or play bag in another terminal):
  ros2 launch gici_ros2 gici_live.launch.py config:=/path/to/rendered_live.yaml

  # Sim time (bag replay with --clock):
  ros2 launch gici_ros2 gici_live.launch.py config:=/path/to/yaml use_sim_time:=true

See scripts/ros2/launch_gici_live.sh and ros2_wrapper/docs/ROS2_LIVE_NODE.md
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
            description='Absolute path to resolved GICI live YAML (replay.enable: false)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (set true when playing rosbag with --clock)',
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
