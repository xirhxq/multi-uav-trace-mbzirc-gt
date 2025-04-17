from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi-uav-trace-mbzirc-gt',
            executable='pose_bridge',
            name='pose_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['10']
        ),
    ])
