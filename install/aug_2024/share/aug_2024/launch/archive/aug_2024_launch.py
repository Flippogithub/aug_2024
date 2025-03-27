import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('aug_2024')
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024.urdf')
    
    # Use xacro to process the file
    #xacro_command = f"xacro {urdf_file} use_sim:=false"
    xacro_command = f"xacro {urdf_file} use_sim:=false"  # Change to false for real hardware
    robot_description_raw = os.popen(xacro_command).read().strip()
    
    # Parse the XML and convert it back to a string
    robot_description_xml = ET.fromstring(robot_description_raw)
    robot_description = ET.tostring(robot_description_xml, encoding='unicode')
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}]
    )

    controller_config = os.path.join(pkg_share, 'config', 'flippo_controllers.yaml')

    controller_manager_node = Node(
     package="controller_manager",
     executable="ros2_control_node",
     parameters=[{'robot_description': robot_description}, controller_config],
     output="screen",
     arguments=['--ros-args', '--log-level', 'debug'],
    )

    def load_controller(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[load_controller("joint_state_broadcaster")],
                ),
            ],
        )
    )

    diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[load_controller("diff_drive_controller")],
                ),
            ],
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])