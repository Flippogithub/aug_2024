import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('aug_2024')
    urdf_file = os.path.join(pkg_path, 'description', 'urdf', 'aug_2024-nocaster.urdf')
    controller_config = os.path.join(pkg_path, 'config', 'flippo_controllers.yaml')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc},
                controller_config
            ]
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad']
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont']
        )
    ])

