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
    
    # Declare the launch arguments - make sure this is declared before any node uses it
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
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

    slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
               'use_sim_time': True,
                'odom_frame': 'odom',  # Match your odometry topic
                'base_frame': 'base_footprint',
                'resolution': 0.05,
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'scan_topic': '/scan'
            }]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                 'robot_description': robot_desc}]  # Use the string directly
)

    # Controller Manager
    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       parameters=[{'robot_description': robot_desc},  # Use the string directly
            os.path.join(pkg_share, 'config', 'flippo_controllers.yaml'),
            {'use_sim_time': use_sim_time}],
       output="screen",
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
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
    
    # Use RegisterEventHandler for joint_broadcaster to start after controller_manager
    joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_node]
        )
    )
    
    # Use TimerAction for diff_drive_node to start a bit after joint_broadcaster
    # This is more reliable than trying to chain event handlers
    diff_drive_spawner = TimerAction(
        period=5.0,  # Start 5 seconds after launch to ensure controller_manager and joint_broadcaster are ready
        actions=[diff_drive_node]
    )
    
    # Use TimerAction for twist_mux to start after diff_drive_node
    twist_mux_spawner = TimerAction(
        period=7.0,  # Start 7 seconds after launch to ensure diff_drive is ready
        actions=[twist_mux_node]
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    teleop_node = Node(
       package='teleop_twist_keyboard',
       executable='teleop_twist_keyboard',
       name='teleop_twist_keyboard',
       output='screen',
       prefix = 'xterm -e',
       remappings=[('/cmd_vel', '/cmd_vel_teleop')],
       parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )   
        
    print(f"URDF file path: {urdf_file}")
    print(f"Robot description length: {len(robot_desc)}")
    
    # RPLidar node only
    rplidar_node = Node(
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
    )
    
    return LaunchDescription([
        # Important: Put declarations FIRST, before any nodes that use them
        declare_use_sim_time_argument,
        declare_map_yaml_cmd,
        rplidar_node,
        robot_state_publisher,
        controller_manager,
        joint_broad_spawner,    # Start joint broadcaster when controller_manager starts
        diff_drive_spawner,     # Start diff_drive controller after a delay
        twist_mux_spawner,      # Start twist_mux after a longer delay
        nav2_launch,
        teleop_node,
        #slam_toolbox,
        static_tf_node,
        rviz
    ])