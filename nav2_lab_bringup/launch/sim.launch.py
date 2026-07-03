import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_world(world_name):
    if os.path.isabs(world_name):
        return world_name

    candidate = world_name
    if not candidate.endswith('.world'):
        candidate = f'{candidate}.world'

    return os.path.join(
        get_package_share_directory('nav2_lab_worlds'),
        'worlds',
        candidate,
    )


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    world = _resolve_world(LaunchConfiguration('world').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    use_gzclient = LaunchConfiguration('use_gzclient')

    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    urdf_path = os.path.join(turtlebot3_gazebo_share, 'urdf', f'turtlebot3_{model}.urdf')
    sdf_path = os.path.join(turtlebot3_gazebo_share, 'models', f'turtlebot3_{model}', 'model.sdf')
    local_gazebo_models = os.path.expanduser('~/gazebo_models')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gazebo_env = {
        'GAZEBO_MODEL_DATABASE_URI': '',
        'GAZEBO_MODEL_PATH': os.pathsep.join([
            os.path.join(turtlebot3_gazebo_share, 'models'),
            local_gazebo_models,
            '/usr/share/gazebo-11/models',
        ]),
        'GAZEBO_PLUGIN_PATH': os.pathsep.join(['/opt/ros/humble/lib']),
        'GAZEBO_RESOURCE_PATH': os.pathsep.join(['/usr/share/gazebo-11']),
    }

    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        ExecuteProcess(
            cmd=[
                'gzserver',
                world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_force_system.so',
            ],
            output='screen',
            additional_env=gazebo_env,
        ),
        ExecuteProcess(
            condition=IfCondition(use_gzclient),
            cmd=['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so'],
            output='screen',
            additional_env=gazebo_env,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', model,
                        '-file', sdf_path,
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.01',
                        '-Y', yaw,
                        '-timeout', '120',
                    ],
                    output='screen',
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room', description='World name or absolute .world path.'),
        DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model.'),
        DeclareLaunchArgument('x_pose', default_value='-1.2', description='Initial robot x position.'),
        DeclareLaunchArgument('y_pose', default_value='-1.2', description='Initial robot y position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo simulation clock.'),
        DeclareLaunchArgument('use_gzclient', default_value='true', description='Start the Gazebo GUI client.'),
        OpaqueFunction(function=launch_setup),
    ])
