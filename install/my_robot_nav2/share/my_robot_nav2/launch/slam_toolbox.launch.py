from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'max_laser_range': 20.0},
                {'resolution': 0.05},
                {'map_update_interval': 5.0},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},  # Adjust if your base frame is different
                {'odom_frame': 'odom'},
                {'scan_topic': 'scan'},  # Adjust if your scan topic is different
            ]
        )
    ])