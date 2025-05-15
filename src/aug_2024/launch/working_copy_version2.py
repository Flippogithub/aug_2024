import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('aug_2024')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024-nocaster.urdf')
    
    # Controller config path
    controllers_config = os.path.join(pkg_share, 'config', 'flippo_controllers.yaml')
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controllers_config
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_broad_spawner",
        arguments=["joint_broad"],  # The name in the YAML file
        output="screen",
    )
    
    # Diff Controller
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="diff_cont_spawner",
        arguments=["diff_cont"],  # The name in the YAML file
        output="screen",
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller
    ])