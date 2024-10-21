from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node: start without delay
        Node(
            package='calibrate',
            executable='clock_node',  # Replace with your script name
            name='clock_node',
            output='screen'
        ),
        # Second node: starts 5 seconds after the first
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                Node(
                    package='calibrate',
                    executable='ouster_node',  # Replace with your script name
                    name='ouster_node',
                    output='screen'
                )
            ]
        ),
        # Third node: starts 10 seconds after the first
        TimerAction(
            period=1.0,  # Delay in seconds
            actions=[
                Node(
                    package='calibrate',
                    executable='zed_node',  # Replace with your script name
                    name='zed_node',
                    output='screen'
                )
            ]
        ),
    ])
