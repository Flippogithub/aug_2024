import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('aug_2024')
    
    # Set up robot description
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024-nocaster.urdf') 
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Controller Manager
    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       name="controller_manager",
       parameters=[
           {'robot_description': robot_desc},
           os.path.join(pkg_share, 'config', 'flippo_controllers.yaml'),
       ],
       output="screen",
    )

        # Joint State Broadcaster spawner
    # Joint State Broadcaster spawner
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Differential drive controller spawner
    diff_drive_spawner = TimerAction(
        period=4.0,  # Start 4 seconds after launch
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_broadcaster_spawner,
        diff_drive_spawner
    ])