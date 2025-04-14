import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('aug_2024')
    urdf_file = os.path.join(pkg_path, 'description', 'urdf', 'aug_2024-nocaster.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc},
            ],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
                '-t', 'joint_state_broadcaster/JointStateBroadcaster'
            ],
            output='screen'
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_cont',
                '--controller-manager', '/controller_manager',
                '-t', 'diff_drive_controller/DiffDriveController',
                '--param', 'left_wheel_names:=["rear_left_wheel_joint"]',
                '--param', 'right_wheel_names:=["rear_right_wheel_joint"]',
                '--param', 'wheel_separation:=0.26',
                '--param', 'wheel_radius:=0.075',
                '--param', 'publish_rate:=50',
                '--param', 'base_frame_id:=base_footprint',
                '--param', 'odom_frame_id:=odom',
                '--param', 'enable_odom_tf:=true'
            ],
            output='screen'
        )
    ])
