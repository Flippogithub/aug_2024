import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    package_name = 'aug_2024'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true')

    # Get URDF via xacro
    pkg_path = os.path.join(get_package_share_directory('aug_2024'))
    xacro_file = os.path.join(pkg_path, 'description', 'urdf', 'aug_2024.xacro')
    robot_description_content = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim:=', 'false',
        ' use_ros2_control:=', use_ros2_control
    ]), value_type=str)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'flippo_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, controller_params_file],
        output="both",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_ros2_control_cmd,
        node_robot_state_publisher,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])