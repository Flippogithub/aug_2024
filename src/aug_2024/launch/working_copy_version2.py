import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('aug_2024')
    config_filepath = LaunchConfiguration('config_filepath', default=os.path.join(pkg_share, 'config', 'twist_mux.yaml'))
    
    # Set up robot description
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024-nocaster.urdf') 
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Keep false for real hardware
        description='Use simulation/Gazebo clock')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'config', 'map1.yaml'),
        description='Full path to map yaml file to load')

    twist_mux_node = Node(
       package='twist_mux',
       executable='twist_mux',
       name='twist_mux',
       parameters=[config_filepath, {'use_sim_time': use_sim_time}],
       remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                 'robot_description': robot_desc}]
    )

    # Controller Manager
    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       parameters=[{'robot_description': robot_desc},
            os.path.join(pkg_share, 'config', 'flippo_controllers.yaml'),
            {'use_sim_time': use_sim_time}],
       output="screen",
    )

    # These transforms are crucial - keeping the explicit transforms
    # Transform from map to odom - critical for navigation
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Transform from odom to base_footprint
    odom_to_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    # Define the joint broadcaster node directly
    joint_broad_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"], 
        output="screen",
    )
    
    # Define the diff controller node directly
    diff_drive_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )
    
    # Use TimerAction for various components to ensure proper start sequence
    joint_broad_spawner = TimerAction(
        period=3.0,  # Start 3 seconds after launch
        actions=[joint_broad_node]
    )
    
    diff_drive_spawner = TimerAction(
        period=8.0,  # Start 8 seconds after launch
        actions=[diff_drive_node]
    )
    
    twist_mux_spawner = TimerAction(
        period=12.0,  # Start 12 seconds after launch
        actions=[twist_mux_node]
    )

    # RPLidar node with modified device path
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 256000,
            'frame_id': 'lidar_link',
            'scan_mode': 'Standard',
            'angle_compensate': True,
            'inverted': False,
        }],
        output='screen'
    )
    
    # Command-line teleop for headless operation
    teleop_node = Node(
       package='teleop_twist_keyboard',
       executable='teleop_twist_keyboard',
       name='teleop_twist_keyboard',
       output='screen',
       remappings=[('/cmd_vel', '/cmd_vel_teleop')],
       parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Nav2 added back, but with a delay to ensure transforms and controllers are ready
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )
    
    # Wrap Nav2 in a TimerAction to ensure it starts after transforms and controllers
    nav2_spawner = TimerAction(
        period=15.0,  # Start 15 seconds after launch to ensure other components are ready
        actions=[nav2_launch]
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
            'video_device': '/dev/video0',  # Check if this is correct for your camera
            'output_encoding': 'rgb8',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ]
    )

# Camera transform publisher - adjust position as needed for your robot
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.2', '0', '0.075', '0', '0', '0', 'base_link', 'camera_link']
    )

    image_transport_node = Node(
        package='image_transport',
        executable='republish',
        name='image_transport_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed')
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

# Group camera nodes in a timer action to start after basic robot functionality
    camera_nodes_spawner = TimerAction(
        period=20.0,  # Start 20 seconds after launch to ensure robot is fully up
        actions=[
            camera_node,
            camera_tf_node,
            image_transport_node
        ]
    )


    return LaunchDescription([
        # Put declarations FIRST
        declare_use_sim_time_argument,
        declare_map_yaml_cmd,
        
        # Critical infrastructure nodes
        robot_state_publisher,
        controller_manager,
        
        # Transform publishers - critical for navigation
        map_to_odom_tf_node,
        odom_to_base_tf_node,
        
        # Hardware nodes with delayed start
        rplidar_node,
        joint_broad_spawner,
        diff_drive_spawner,
        twist_mux_spawner,
        
        # Nav2 with a delay to ensure prerequisites are ready
        nav2_spawner,
        camera_nodes_spawner,
        # Teleop for basic testing
        teleop_node,
    ])
