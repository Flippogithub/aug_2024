import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('aug_2024')
    
    # Set up robot description
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024-nocaster.urdf') 
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Search for the joint names in the URDF
    import re
    joint_pattern = re.compile(r'<joint\s+name="([^"]+)"')
    joints = joint_pattern.findall(robot_desc)
    print(f"Found joints in URDF: {joints}")
    
    # Just return a basic launch description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    return LaunchDescription([robot_state_publisher])