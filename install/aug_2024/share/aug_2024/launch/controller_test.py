import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF via xacro
    pkg_path = get_package_share_directory('aug_2024')
    urdf_file = os.path.join(pkg_path, 'description', 'urdf', 'aug_2024-nocaster.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launch Description
    return LaunchDescription([
        # Parameters
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation clock if true'),

        # Nodes
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }]
        ),
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
                {
                    'controller_manager': {
                        'update_rate': 50
                    },
                    'joint_state_broadcaster': {
                        'type': 'joint_state_broadcaster/JointStateBroadcaster'
                    },
                    'diff_cont': {
                        'type': 'diff_drive_controller/DiffDriveController',
                        'left_wheel_names': ['rear_left_wheel_joint'],
                        'right_wheel_names': ['rear_right_wheel_joint'],
                        'publish_rate': 50,
                        'base_frame_id': 'base_footprint',
                        'odom_frame_id': 'odom',
                        'enable_odom_tf': True
                    }
                }
            ],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
