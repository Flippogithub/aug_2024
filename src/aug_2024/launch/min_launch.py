import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create minimal URDF
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

    # Controller manager with parameters inline
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            {'robot_description': minimal_urdf},
            {'controller_manager': {
                'ros__parameters': {
                    'update_rate': 50,
                    'joint_broad': {
                        'type': 'joint_state_broadcaster/JointStateBroadcaster'
                    },
                    'diff_cont': {
                        'type': 'diff_drive_controller/DiffDriveController'
                    }
                }
            }},
            {'diff_cont': {
                'ros__parameters': {
                    'left_wheel_names': ['rear_left_wheel_joint'],
                    'right_wheel_names': ['rear_right_wheel_joint'],
                    'publish_rate': 50,
                    'base_frame_id': 'base_footprint',
                    'odom_frame_id': 'odom',
                    'enable_odom_tf': True
                }
            }}
        ],
        output="screen"
    )
    
    return LaunchDescription([controller_manager])