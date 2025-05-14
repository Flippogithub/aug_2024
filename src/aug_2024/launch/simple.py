import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    
    # Declare the launch arguments - all declarations come first
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Changed to false for hardware
        description='Use simulation/Gazebo clock')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'config', 'map1.yaml'),
        description='Full path to map yaml file to load')

    declare_joy_device = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path')

    # IMPORTANT: Map frame first
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Robot state publisher - the foundation for TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                    'use_sim_time': use_sim_time}]
    )

    # TF tree setup - static transforms
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']
    )

    # Controller Manager - with reduced update rate for Pi
    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       name="controller_manager",  # Explicit name to avoid duplicates
       parameters=[{'robot_description': robot_desc},
            os.path.join(pkg_share, 'config', 'flippo_controllers.yaml'),
            {'use_sim_time': use_sim_time,
             'update_rate': 10}],  # Lower rate for Raspberry Pi
       output="screen",
    )

    # RPLidar node
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

    # Controller spawners - using TimerAction for better sequencing
    joint_broadcaster_spawner = TimerAction(
        period=3.0,  # Delay for controller_manager to fully initialize
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawner_joint_state_broadcaster",
                arguments=["joint_state_broadcaster", 
                           "--controller-manager", "/controller_manager",
                           "--controller-manager-timeout", "60"],  # FIXED parameter name
                output="screen",
            )
        ]
    )
    
    diff_drive_spawner = TimerAction(
        period=6.0,  # Longer delay to ensure joint_state_broadcaster is running
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawner_diff_controller",
                arguments=["diff_cont", 
                          "--controller-manager", "/controller_manager",
                          "--controller-manager-timeout", "60"],  # FIXED parameter name
                output="screen",
            )
        ]
    )

    # Control input nodes with appropriate delays
    joy_node = TimerAction(
        period=7.0,  # Start after controllers
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'device': LaunchConfiguration('joy_device'),
                    'deadzone': 0.1,
                    'autorepeat_rate': 20.0,
                }],
                output='screen'
            )
        ]
    )

    teleop_joy_node = TimerAction(
        period=8.0,  # Start after joy node
        actions=[
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[{
                    'axis_linear.x': 1,
                    'axis_angular.yaw': 0,
                    'scale_linear.x': 0.5,
                    'scale_angular.yaw': 0.5,
                    'enable_button': 4,
                }],
                remappings=[('/cmd_vel', '/cmd_vel_teleop')],
                output='screen'
            )
        ]
    )

    # Twist multiplexer with delay
    twist_mux_spawner = TimerAction(
        period=10.0,  # Start after teleop is ready
        actions=[
            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                parameters=[config_filepath, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
            )
        ]
    )

    # Navigation stack with longer delay for resource-constrained Pi
    nav2_launch = TimerAction(
        period=15.0,  # Start after all controllers and hardware are ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
                }.items()
            )
        ]
    )
    
    print(f"URDF file path: {urdf_file}")
    print(f"Robot description length: {len(robot_desc)}")
    
    return LaunchDescription([
        # Declarations first
        declare_use_sim_time_argument,
        declare_map_yaml_cmd,
        declare_joy_device,
        
        # Critical TF tree setup first
        map_to_odom_tf,  # THIS IS CRITICAL
        robot_state_publisher,
        odom_to_base_tf,
        camera_tf_node,
        
        # Hardware interface
        controller_manager,
        rplidar_node,
        
        # Controllers with sequential timing
        joint_broadcaster_spawner,
        diff_drive_spawner,
        
        # Control inputs
        joy_node,
        teleop_joy_node,
        twist_mux_spawner,
        
        # Navigation stack (last, after everything else is ready)
        nav2_launch
    ])
