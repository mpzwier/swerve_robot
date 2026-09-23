import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    # Package paths
    pkg_share=get_package_share_directory('multiple_robots')
    ros_gz_sim_share=get_package_share_directory('ros_gz_sim')
    nav2_share=get_package_share_directory('nav2_bringup')


    # File paths
    world_path=os.path.join(pkg_share, 'world', 'simple_world.sdf')
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    dock_locations_path=os.path.join(pkg_share, 'config', 'dock_locations.yaml')

    # Load dock database from YAML
    with open(dock_locations_path, 'r') as f:
        dock_database_dict = yaml.safe_load(f)

    # Robots information

    robots = [
        {
            'name': 'robot1',
            'controller_type': 'swerve_drive_controller',
            'x': '0.75',
            'y': '0.75',
            'yaw': '1.5708',
            'model': os.path.join(pkg_share, 'description', 'robot1_swerve_bot.urdf'),
            'controller': os.path.join(pkg_share, 'config', 'robot1_swerve_drive_controllers_params.yaml'),
            'nav2': os.path.join(pkg_share, 'config', 'robot1_nav2_params.yaml'),
            'ekf': os.path.join(pkg_share, 'config', 'robot1_ekf.yaml'),
            'bridge': os.path.join(pkg_share, 'config', 'robot1_bridge_config.yaml'),
            'rviz': os.path.join(pkg_share, 'rviz', 'robot1_config.rviz'), 
            'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),
            'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_swerve_drive.yaml')
        },
        # {
        #     'name': 'robot2',
        #     'controller_type': 'swerve_drive_controller',
        #     'x': '2.45',
        #     'y': '0.75',
        #     'yaw': '1.5708',
        #     'model': os.path.join(pkg_share, 'description', 'robot2_swerve_bot.urdf'),
        #     'controller': os.path.join(pkg_share, 'config', 'robot2_swerve_drive_controllers_params.yaml'),
        #     'nav2': os.path.join(pkg_share, 'config', 'robot2_nav2_params.yaml'),
        #     'ekf': os.path.join(pkg_share, 'config', 'robot2_ekf.yaml'),
        #     'bridge': os.path.join(pkg_share, 'config', 'robot2_bridge_config.yaml'),
        #     'rviz': os.path.join(pkg_share, 'rviz', 'robot2_config.rviz'), 
        #     'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),   
        #     'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_swerve_drive.yaml')        
        # },
        # {
        #     'name': 'robot3',
        #     'controller_type': 'swerve_drive_controller',
        #     'x': '3.90',
        #     'y': '0.5',
        #     'yaw': '1.5708',
        #     'model': os.path.join(pkg_share, 'description', 'robot3_swerve_bot.urdf'),
        #     'controller': os.path.join(pkg_share, 'config', 'robot3_swerve_drive_controllers_params.yaml'),
        #     'nav2': os.path.join(pkg_share, 'config', 'robot3_nav2_params.yaml'),
        #     'ekf': os.path.join(pkg_share, 'config', 'robot3_ekf.yaml'),
        #     'bridge': os.path.join(pkg_share, 'config', 'robot3_bridge_config.yaml'),
        #     'rviz': os.path.join(pkg_share, 'rviz', 'robot3_config.rviz'), 
        #     'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),
        #     'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_swerve_drive.yaml')          
        # },
        # {
        #     'name': 'robot4',
        #     'controller_type': 'diff_drive_controller',
        #     'x': '5.1',
        #     'y': '0.5',
        #     'yaw': '1.5708',
        #     'model': os.path.join(pkg_share, 'description', 'robot4_diff_bot.urdf'),
        #     'controller': os.path.join(pkg_share, 'config', 'robot4_diff_drive_controllers_params.yaml'),
        #     'nav2': os.path.join(pkg_share, 'config', 'robot4_nav2_params.yaml'),
        #     'ekf': os.path.join(pkg_share, 'config', 'robot4_ekf.yaml'),
        #     'bridge': os.path.join(pkg_share, 'config', 'robot4_bridge_config.yaml'),
        #     'rviz': os.path.join(pkg_share, 'rviz', 'robot4_config.rviz'), 
        #     'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),
        #     'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_diff_drive.yaml'),
        #     'stamped': os.path.join(pkg_share, 'config', 'stamped.yaml')          
        # },
        # {
        #     'name': 'robot5',
        #     'controller_type': 'diff_drive_controller',
        #     'x': '6.25',
        #     'y': '0.5',
        #     'yaw': '1.5708',
        #     'model': os.path.join(pkg_share, 'description', 'robot5_diff_bot.urdf'),
        #     'controller': os.path.join(pkg_share, 'config', 'robot5_diff_drive_controllers_params.yaml'),
        #     'nav2': os.path.join(pkg_share, 'config', 'robot5_nav2_params.yaml'),
        #     'ekf': os.path.join(pkg_share, 'config', 'robot5_ekf.yaml'),
        #     'bridge': os.path.join(pkg_share, 'config', 'robot5_bridge_config.yaml'),
        #     'rviz': os.path.join(pkg_share, 'rviz', 'robot5_config.rviz'), 
        #     'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),
        #     'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_diff_drive.yaml'),
        #     'stamped': os.path.join(pkg_share, 'config', 'stamped.yaml')          
        # },
        # {
        #     'name': 'robot6',
        #     'controller_type': 'diff_drive_controller',
        #     'x': '7.5',
        #     'y': '0.5',
        #     'yaw': '1.5708',
        #     'model': os.path.join(pkg_share, 'description', 'robot6_diff_bot.urdf'),
        #     'controller': os.path.join(pkg_share, 'config', 'robot6_diff_drive_controllers_params.yaml'),
        #     'nav2': os.path.join(pkg_share, 'config', 'robot6_nav2_params.yaml'),
        #     'ekf': os.path.join(pkg_share, 'config', 'robot6_ekf.yaml'),
        #     'bridge': os.path.join(pkg_share, 'config', 'robot6_bridge_config.yaml'),
        #     'rviz': os.path.join(pkg_share, 'rviz', 'robot6_config.rviz'), 
        #     'map': os.path.join(pkg_share, 'config', 'robots_map.yaml'),
        #     'scan_mask': os.path.join(pkg_share, 'config', 'scan_mask_diff_drive.yaml'),
        #     'stamped': os.path.join(pkg_share, 'config', 'stamped.yaml')          
        # },
    ]

    # Gazebo

    start_gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f"-r {world_path}"
        }.items(),      
    )

    clock=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Multi robot launch
  
    robot_groups=[]
    for robot in robots:

        namespace=robot['name']

        # RewrittenYaml merges it into the docking_server params
        nav2_params = RewrittenYaml(
        source_file=robot['nav2'],
        param_rewrites={'docking_server.dock_database': dock_database_dict},
        convert_types=True
        )      
       
        # Build actions list first (so we can conditionally add nodes)
        robot_actions=[

            # GZ-ROS bridge
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                parameters=[{
                    'config_file': robot['bridge'],
                    'expand_gz_topic_names': True,
                    'use_sim_time': True,
                }],
            ),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                namespace=namespace,
                parameters=[{
                    'robot_description': ParameterValue(Command([PathJoinSubstitution([FindExecutable(name='xacro')]),' ',robot['model'],]),value_type=str),
                    'use_sim_time': True,
                }],
                remappings=remappings,
            ),

            # Load model in GZ
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic', f'/{namespace}/robot_description', 
                    '-name', robot['name'],
                    '-x', robot['x'],
                    '-y', robot['y'],
                    '-Y', robot['yaw']],
            ),

            # Lidar scan filtered
            Node(
                package='scan_mask_filter',
                executable='scan_mask_node',
                namespace=namespace,
                output='screen',
                parameters=[robot['scan_mask']],
            ),
        ]

        # TwistStamped (only for diff_drive)
        if robot.get('controller_type') == 'diff_drive_controller':
            robot_actions.append(
                Node(
                    package='stamped_filter',
                    executable='stamped_filter_node',
                    namespace=namespace,
                    output='screen',
                    parameters=[robot['stamped']]
                )
            )

        robot_actions.extend([

            # Localization
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                namespace=namespace,
                parameters=[
                    #'params', robot['ekf'],
                    robot['ekf'],
                    {'use_sim_time': True},
                ],
                remappings=remappings
            ),

            # Controllers
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                namespace=namespace,
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    f'/{namespace}/controller_manager',
                    ],
            ),
            

            Node(
                package='controller_manager',
                executable='spawner',
                output="screen",
                namespace=namespace,
                arguments=[
                    robot['controller_type'], 
                    '--controller-manager',
                    f'/{namespace}/controller_manager',
                    '--param-file',
                    robot['controller'],
                ],
            ),

            # Rivz
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                namespace=namespace,
                arguments=['-d', robot['rviz']],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose'),
                ],  
            ),

            # Nav2 delayed
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
                        ),
                        launch_arguments={
                            'use_sim_time': 'True',
                            'autostart': 'True',
                            'map': robot['map'],
                            'params_file': nav2_params, #robot['nav2'],
                            'namespace': namespace,
                            'use_namespace': 'True',
                            'use_composition': 'True',
                        }.items(),
                    )
                ]
            ),
        ])

        robot_groups.append(GroupAction(robot_actions))
   


  
    return LaunchDescription([
            

        #Launching Description Nodes, Rviz and Gazebo
        start_gazebo,
        clock,
        *robot_groups,
        
    ])




