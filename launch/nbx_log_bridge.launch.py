"""Launch the NBX Lemma log bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare bounded settings and launch the bridge."""
    history_depth = LaunchConfiguration('history_depth')
    max_message_bytes = LaunchConfiguration('max_message_bytes')

    return LaunchDescription([
        DeclareLaunchArgument(
            'history_depth',
            default_value='500',
            description=(
                'Number of warning-or-higher records retained in memory; '
                'combined retention is limited to 64 MiB.'),
        ),
        DeclareLaunchArgument(
            'max_message_bytes',
            default_value='8192',
            description=(
                'Maximum UTF-8 bytes retained in each Log.msg field; '
                'combined retention is limited to 64 MiB.'),
        ),
        Node(
            package='nbx_log_bridge',
            executable='nbx_log_bridge',
            name='nbx_log_bridge',
            output='screen',
            parameters=[{
                'history_depth': ParameterValue(history_depth, value_type=int),
                'max_message_bytes': ParameterValue(
                    max_message_bytes, value_type=int),
            }],
        ),
    ])
