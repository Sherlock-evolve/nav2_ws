import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('nav2_lab_bringup')
    missions_share = get_package_share_directory('nav2_lab_missions')

    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    mission_file = LaunchConfiguration('mission_file')
    run_mission = LaunchConfiguration('run_mission')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room', description='World name or absolute .world path.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_share, 'maps', 'simple_room.yaml'),
            description='Full path to the map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'params', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'mission_file',
            default_value=os.path.join(missions_share, 'config', 'simple_room_mission.yaml'),
            description='Mission YAML file.',
        ),
        DeclareLaunchArgument('run_mission', default_value='false', description='Run configured mission automatically.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
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
                ),
                Node(
                    condition=IfCondition(run_mission),
                    package='nav2_lab_missions',
                    executable='mission_logger',
                    name='mission_logger',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ],
        ),
    ])
