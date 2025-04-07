import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('aug_2024')
    config_filepath = os.path.join(pkg_share, 'config', 'flippo_controllers.yaml')
    
    # Create a minimal URDF with mock hardware
    minimal_urdf = """<?xml version="1.0"?>
<robot name="minimal_robot">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="rear_left_wheel_link"/>
  <link name="rear_right_wheel_link"/>
  
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
  </joint>
  
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <ros2_control name="MockSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
    
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': minimal_urdf},
            config_filepath
        ],
        output="screen",
    )
    
    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )
    
    # Diff drive controller spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )
    
    return LaunchDescription([
        controller_manager,
        joint_broad_spawner,
        diff_drive_spawner
    ])