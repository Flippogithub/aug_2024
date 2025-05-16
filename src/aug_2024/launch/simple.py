import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Explicitly set paths to files
    urdf_file = "/home/rospi/aug_2024/src/aug_2024/description/urdf/aug_2024-nocaster.urdf"
    controller_config = "/home/rospi/aug_2024/src/aug_2024/config/flippo_controllers.yaml"
    
    print(f"URDF Path: {urdf_file}")
    print(f"Config Path: {controller_config}")
    print(f"URDF Exists: {os.path.exists(urdf_file)}")
    print(f"Config Exists: {os.path.exists(controller_config)}")
    
    # Debug YAML parsing
    try:
        with open(controller_config, 'r') as file:
            yaml_content = file.read()
            print("Raw YAML Content:")
            print(yaml_content)
            
            parsed_yaml = yaml.safe_load(yaml_content)
            print("Parsed YAML:")
            print(parsed_yaml)
    except Exception as e:
        print(f"Error parsing YAML: {e}")
        sys.exit(1)
    
    # Read robot description using xacro
    try:
        robot_description = xacro.process_file(urdf_file).toxml()
        print(f"Successfully processed URDF file with xacro, length: {len(robot_description)}")
    except Exception as e:
        print(f"Error processing URDF with xacro: {e}")
        # Fallback to direct file reading
        try:
            with open(urdf_file, 'r') as file:
                robot_description = file.read()
                print(f"Read URDF file directly, length: {len(robot_description)}")
        except Exception as e:
            print(f"Error reading URDF file: {e}")
            sys.exit(1)
    
    # Nodes to launch
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
        
        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_config
            ],
            output='screen'
        ),
        
        # Joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
            output='screen'
        ),

         # RPLidar node only
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 256000,
                'frame_id': 'lidar_link',  # Make sure this matches your URDF link name
                'scan_mode': 'Standard',
                'angle_compensate': True,
                'inverted': False,
            }],
            output='screen'
        ),
        
        # Differential drive controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen'
        )
    ])
