import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('aug_2024')
    config_filepath = LaunchConfiguration('config_filepath', default=os.path.join(pkg_share, 'config', 'twist_mux.yaml'))
    # Set up robot description
    world_file_path = os.path.join(pkg_share, 'worlds', 'usda_rough_dim.sdf')
    urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'aug_2024-nocaster.urdf') #was aug_2024.urdf
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'config', 'map1.yaml'),
        description='Full path to map yaml file to load')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
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
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                '-entity', 'aug_2024_bot',
                '-x', '17', '-y', '9', '-z', '0.1'],  # Raised slightly off the ground
        parameters=[{'use_sim_time': use_sim_time}], 
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': Command(['xacro ', urdf_file])}]
    )

    # Diff drive controller spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': use_sim_time}], 
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': use_sim_time}], 
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

    controller_manager = Node(
       package="controller_manager",
       executable="ros2_control_node",
       parameters=[{'robot_description': Command(['xacro ', urdf_file])},
                os.path.join(pkg_share, 'config', 'my_controllers.yaml'),
                {'use_sim_time': use_sim_time} ],
       output="screen",
    )

    delay_controller_spawner = RegisterEventHandler(
       event_handler=OnProcessExit(
       target_action=controller_manager,
       on_exit=[diff_drive_spawner, joint_broad_spawner],
       )
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
    print(f"URDF file path: {urdf_file}")
    print(f"Robot description length: {len(robot_desc)}")
    
    from launch.actions import TimerAction

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )   
        
    waypoint_nav = Node(
         package='aug_2024',  # Change to your package name
         executable='waypoint_navigator',
         name='waypoint_navigator',
         output='screen'
    )

    return LaunchDescription([
        gazebo,
        declare_use_sim_time_argument,
        declare_map_yaml_cmd,
        robot_state_publisher,
        controller_manager,
        delay_controller_spawner,
        twist_mux_node,
        delayed_spawn,  # Use the delayed spawn instead
        nav2_launch,
        teleop_node,
        #slam_toolbox,
        rviz,
        waypoint_nav
    ])