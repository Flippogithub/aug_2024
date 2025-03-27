import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='aug_2024').find('aug_2024')
    default_model_path = os.path.join(pkg_share, 'description/urdf/aug_2024.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    config_filepath = LaunchConfiguration('config_filepath', default=os.path.join(pkg_share, 'config', 'twist_mux.yaml'))
    gazebo_ros2_control_params = os.path.join(pkg_share, 'config', 'gazebo_ros2_control_params.yaml')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl_config.yaml')
    controller_params = os.path.join(pkg_share, 'config', 'flippo_controllers.yaml')
    
    robot_state_publisher_node = launch_ros.actions.Node(
       package='robot_state_publisher',
       executable='robot_state_publisher',
       parameters=[
          {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
          gazebo_ros2_control_params,
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
       ]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
   )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'aug_2024_bot', '-topic', 'robot_description'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
   )

    #robot_localization_node = launch_ros.actions.Node(
     #   package='robot_localization',
      #  executable='ekf_node',
       # name='ekf_filter_node',
        #output='screen',
        #parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    #)

    #controller_manager_node = Node(
     #  package="controller_manager",
      # executable="ros2_control_node",
       #parameters=[
        #  {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
         # controller_params,
          #gazebo_ros2_control_params,
          #{'use_sim_time': LaunchConfiguration('use_sim_time')}
       #],
      # output="both",
    #)

    amcl_node = Node(
       package='nav2_amcl',
       executable='amcl',
       name='amcl',
       output='screen',
       parameters=[amcl_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    twist_mux_node = Node(
       package='twist_mux',
       executable='twist_mux',
       name='twist_mux',
       parameters=[config_filepath, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       remappings=[('/cmd_vel_out','/flippo/cmd_vel')]
    )

    teleop_node = Node(
       package='teleop_twist_keyboard',
       executable='teleop_twist_keyboard',
       name='teleop_twist_keyboard',
       output='screen',
       prefix = 'xterm -e',
       remappings=[('/cmd_vel', '/flippo/cmd_vel')],
       parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    map_server_node = Node(
       package='nav2_map_server',
       executable='map_server',
       name='map_server',
       output='screen',
       parameters=[{'yaml_filename': os.path.join(pkg_share, 'config', 'map1.yaml')},
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    lifecycle_manager_node = Node(
       package='nav2_lifecycle_manager',
       executable='lifecycle_manager',
       name='lifecycle_manager_localization',
       output='screen',
       parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}]
    )
    

    static_tf = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_transform_publisher',
       output='screen',
       arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
       parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )



    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    delay_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner, joint_broad_spawner],
        )
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        

        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
    

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--pause'],
            output='screen'
        ),

        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
    
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        delay_controller_spawner,
        #robot_localization_node,
        rviz_node,
        twist_mux_node,
        teleop_node,
        amcl_node,
        map_server_node,
        lifecycle_manager_node,
        static_tf
        
    ])