import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config_file(value, package_share, directory, suffix):
    expanded = os.path.expanduser(value)
    if os.path.isabs(expanded):
        return expanded

    candidate = expanded
    if suffix and not candidate.endswith(suffix):
        candidate = f'{candidate}{suffix}'

    installed_file = os.path.join(package_share, directory, candidate)
    if os.path.exists(installed_file):
        return installed_file

    package_xml = os.path.join(package_share, 'package.xml')
    if os.path.islink(package_xml):
        source_package = os.path.dirname(os.path.realpath(package_xml))
        source_file = os.path.join(source_package, directory, candidate)
        if os.path.exists(source_file):
            return source_file

    return installed_file


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('nav2_lab_bringup')
    missions_share = get_package_share_directory('nav2_lab_missions')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    mission = LaunchConfiguration('mission').perform(context)
    mission_file_override = LaunchConfiguration('mission_file').perform(context)
    mission_file = _resolve_config_file(
        mission_file_override or mission,
        missions_share,
        'config',
        '.yaml',
    )
    slam = LaunchConfiguration('slam')
    run_mission = LaunchConfiguration('run_mission')
    shutdown_on_mission_complete = LaunchConfiguration('shutdown_on_mission_complete')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    mission_runner_node = Node(
        condition=IfCondition(run_mission),
        package='nav2_lab_missions',
        executable='mission_runner',
        name='mission_runner',
        output='screen',
        parameters=[{
            'mission_file': mission_file,
            'use_sim_time': use_sim_time,
            'nav_activation_delay_sec': 5.0,
            'wait_for_amcl_timeout_sec': 30.0,
            'retry_backoff_sec': 2.0,
        }],
    )
    mission_logger_node = Node(
        condition=IfCondition(run_mission),
        package='nav2_lab_missions',
        executable='mission_logger',
        name='mission_logger',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': world,
                'model': model,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'slam': slam,
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
        ),
        TimerAction(
            period=15.0,
            actions=[
                mission_runner_node,
                mission_logger_node,
            ],
        ),
        RegisterEventHandler(
            condition=IfCondition(shutdown_on_mission_complete),
            event_handler=OnProcessExit(
                target_action=mission_runner_node,
                on_exit=[
                    Shutdown(reason='mission_runner completed'),
                ],
            ),
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('nav2_lab_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room', description='World name or absolute .world path.'),
        DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model.'),
        DeclareLaunchArgument('x_pose', default_value='-1.2', description='Initial robot x position.'),
        DeclareLaunchArgument('y_pose', default_value='-1.2', description='Initial robot y position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument(
            'map',
            default_value='simple_room',
            description='Map name under nav2_lab_bringup/maps or absolute .yaml path.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'params', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'mission',
            default_value='simple_room_mission',
            description='Mission name under nav2_lab_missions/config, without .yaml.',
        ),
        DeclareLaunchArgument(
            'mission_file',
            default_value='',
            description='Optional mission YAML name or path. Overrides mission when set.',
        ),
        DeclareLaunchArgument('slam', default_value='False', description='Run Nav2 with SLAM instead of AMCL.'),
        DeclareLaunchArgument('run_mission', default_value='false', description='Run configured mission automatically.'),
        DeclareLaunchArgument(
            'shutdown_on_mission_complete',
            default_value='false',
            description='Shutdown the whole launch when mission_runner exits.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        OpaqueFunction(function=launch_setup),
    ])
