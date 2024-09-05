import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('aug_2024')
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') #new
    # Use xacro to process the file
    xacro_command = f"xacro {urdf_file} use_sim:=true"
    robot_description_raw = os.popen(xacro_command).read().strip()
    
    declare_use_sim_time_cmd = DeclareLaunchArgument( #new
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    # Parse the XML and convert it back to a string
    robot_description_xml = ET.fromstring(robot_description_raw)
    robot_description = ET.tostring(robot_description_xml, encoding='unicode')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub_node,
        spawn_entity,
        rviz_cmd,
        TimerAction(
            period=35.0,
            actions=[
                load_joint_state_broadcaster,
                TimerAction(
                    period=2.0,
                    actions=[load_diff_drive_controller],
                )
            ]
        )
    ])