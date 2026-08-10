"""Launch live LiDAR and V4L2 camera publishers."""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the live Ouster and generic V4L2 camera publishers."""
    return LaunchDescription([
        # Live sensors use the PTP-disciplined system clock. Do not publish
        # /clock unless running a simulator or rosbag with use_sim_time=true.
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='sensors',
                    executable='ouster_node',
                    name='ouster_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                )
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='sensors',
                    executable='camera_node',
                    name='camera_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                )
            ],
        ),
    ])
