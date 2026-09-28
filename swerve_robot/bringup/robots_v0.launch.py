import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from nav2_common.launch import RewrittenYaml


# Text that nav2's lifecycle_manager_navigation logs (via /rosout) once it
# has finished activating every managed node and creates its bond with the
# lifecycle manager. We treat this as "this robot's Nav2 stack is fully up"
# and use it to gate the start of the next robot.
#
# NOTE: run a single robot first and `ros2 topic echo /rosout` (or grep its
# console output) to confirm the exact wording your nav2_bringup version
# logs, and adjust BOND_TIMER_MARKER below if it differs.
BOND_TIMER_MARKER = 'Creating bond timer'


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('swerve_robot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    # File paths
    world_path = os.path.join(pkg_share, 'world', 'simple_world.sdf')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    dock_locations_path = os.path.join(pkg_share, 'config', 'dock_locations.yaml')

    # Load dock database from YAML
    with open(dock_locations_path, 'r') as f:
        dock_database_dict = yaml.safe_load(f)

    # Robots information
    # NOTE: swerve robots use the *_nav2_params_double.yaml configs (dual
    # controller: RegulatedPurePursuit for the approach, Omni MPPI for the
    # last stretch/docking - see nav_modes.xml), so they can actually use
    # their sideways (y) degree of freedom instead of driving like a
    # diff-drive robot.

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
    ]

    # Gazebo

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f"-r {world_path}"
        }.items(),
    )

    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Multi robot launch
    #
    # Startup ordering: robots are launched strictly one after another.
    # Robot N+1's whole startup sequence (bridge, robot_state_publisher,
    # model spawn, controllers, EKF, RViz, Nav2) is only kicked off once
    # robot N's Nav2 (lifecycle_manager_navigation) has printed the
    # BOND_TIMER_MARKER line to stdout, i.e. once robot N's Nav2 bringup has
    # actually finished - rather than guessing a fixed delay per robot.
    #
    # Per-robot ordering is unchanged: Nav2 for a given robot still only
    # starts once that robot's own drive controller (not just
    # joint_state_broadcaster) has spawned, via an OnProcessExit handler.

    def build_robot_actions(robot):
        """Build every launch action needed to bring one robot online:
        GZ-ROS bridge, robot_state_publisher, model spawn, scan filter,
        (optional) TwistStamped filter, EKF, controller spawners, RViz, and
        Nav2 (started once this robot's drive controller has spawned).
        """
        namespace = robot['name']

        robot_actions = [

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
                    'robot_description': ParameterValue(Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', robot['model'], ]), value_type=str),
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
                    '--controller-manager-timeout',
                    '60',
                ],
            ),
        ])

        # This spawner is kept as its own variable so we can key the Nav2
        # start off its completion (OnProcessExit) below.
        controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output="screen",
            namespace=namespace,
            arguments=[
                robot['controller_type'],
                '--controller-manager',
                f'/{namespace}/controller_manager',
                '--controller-manager-timeout',
                '60',
                '--param-file',
                robot['controller'],
            ],
        )
        robot_actions.append(controller_spawner)

        robot_actions.append(
            # Rviz
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
            )
        )

        # Nav2 for this robot: start only after its own controller has
        # finished spawning, instead of a fixed guessed delay.
        nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'autostart': 'True',
                'map': robot['map'],
                'params_file': robot['nav2'],
                'namespace': namespace,
                'use_namespace': 'True',
                'use_composition': 'True',
            }.items(),
        )

        start_nav2_after_controller = RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawner,
                on_exit=[nav2_include],
            )
        )
        robot_actions.append(start_nav2_after_controller)

        return robot_actions

    def build_bond_wait_process(namespace):
        """A tiny helper process that subscribes to /rosout and exits the
        moment it sees this robot's Nav2 lifecycle manager log the
        BOND_TIMER_MARKER line (i.e. that robot's Nav2 bringup has fully
        finished).

        We watch /rosout (a normal ROS topic that RCLCPP_INFO etc. publish
        to) instead of the process' raw stdout, because stdout capture
        depends on each Node's `output=` setting and on whether Nav2 is
        running composed inside a container - /rosout sidesteps all of
        that as long as rosout logging isn't disabled.
        """
        script = f'''
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

NAMESPACE_PREFIX = "{namespace}."
MARKER = "{BOND_TIMER_MARKER}"

class BondWaiter(Node):
    def __init__(self):
        super().__init__("bond_timer_waiter_{namespace}")
        self.create_subscription(Log, "/rosout", self.cb, 50)
        self.get_logger().info(f"waiting for '{{MARKER}}' from nodes under {{NAMESPACE_PREFIX}}*")

    def cb(self, msg):
        if not msg.name.startswith(NAMESPACE_PREFIX):
            return
        # DEBUG: print every rosout line from this robot's nodes so we can
        # see the exact wording/name if MARKER never matches. Remove once
        # the match is confirmed working.
        print(f"[bond_wait:{namespace}] name={{msg.name}} msg={{msg.msg}}", flush=True)
        if MARKER in msg.msg:
            print(f"[bond_wait:{namespace}] MATCHED - releasing next robot", flush=True)
            rclpy.shutdown()
            sys.exit(0)

def main():
    rclpy.init()
    rclpy.spin(BondWaiter())

if __name__ == "__main__":
    main()
'''


        
        return ExecuteProcess(
            cmd=['python3', '-c', script],
            name=f'bond_timer_waiter_{namespace}',
            output='screen',
        )

    def launch_robot(index):
        """Return the actions that bring up robots[index], and - unless
        it's the last robot - the actions for robots[index + 1] wired to
        start only once a bond-wait helper confirms robots[index]'s Nav2
        stack has finished bringup. This chains the whole fleet so each
        robot only starts after the previous one is actually up.
        """
        if index >= len(robots):
            return []

        robot = robots[index]
        actions = build_robot_actions(robot)

        if index + 1 < len(robots):
            waiter = build_bond_wait_process(robot['name'])
            actions.append(waiter)
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=waiter,
                        on_exit=launch_robot(index + 1),
                    )
                )
            )

        return actions

    return LaunchDescription([

        # Launching Description Nodes, Rviz and Gazebo
        start_gazebo,
        clock,
        *launch_robot(0),

    ])
