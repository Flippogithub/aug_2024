import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
    
    # Declare joy device parameter
    declare_joy_device = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    # Joy node for PS4 controller - CORRECTED parameter name
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': LaunchConfiguration('joy_device'),  # Changed from device_name to device
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Teleop twist joy node
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,  # Left stick vertical
            'axis_angular.yaw': 0,  # Left stick horizontal
            'scale_linear.x': 0.5,
            'scale_angular.yaw': 0.5,
            'enable_button': 4,  # L1 button (may need adjustment)
        }],
        remappings=[('/cmd_vel', '/cmd_vel_teleop')],  # Match your existing teleop remapping
        output='screen'
    )

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
   # Controller Manager
    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       name="controller_manager",  # Add explicit name
       parameters=[{'robot_description': robot_desc},
            os.path.join(pkg_share, 'config', 'flippo_controllers.yaml'),
            {'use_sim_time': use_sim_time}],
       output="screen",
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    joint_broad_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"], 
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
    joint_broadcaster_spawner = RegisterEventHandler(
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
    
    # Add v4l2_camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',  # Changed from MJPG to YUYV which is better supported
            'video_device': '/dev/video0',
            'output_encoding': 'rgb8'  # Explicitly set the output encoding
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ]
    )
    
    # Add camera TF broadcaster (assuming camera is mounted on the robot)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']  # Adjust position as needed
    )
    
    # Optional: Add image_transport republisher for compressed images
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
    
    return LaunchDescription([
        # Important: Put declarations FIRST, before any nodes that use them
        declare_use_sim_time_argument,
        declare_map_yaml_cmd,
        declare_joy_device,  # ADDED THIS LINE - Declaration for joy_device
        rplidar_node,
        robot_state_publisher,
        controller_manager,
        joint_broadcaster_spawner,    # Start joint broadcaster when controller_manager starts
        diff_drive_spawner,     # Start diff_drive controller after a delay
        twist_mux_spawner,      # Start twist_mux after a longer delay
        nav2_launch,
        teleop_node,
        joy_node,  # Add PS4 controller node
        teleop_joy_node,  # Add PS4 teleop node
        #slam_toolbox,
        static_tf_node,
        camera_node,            # Add camera node
        camera_tf_node,         # Add camera transform
        image_transport_node,   # Add image compression
        #rviz
    ])