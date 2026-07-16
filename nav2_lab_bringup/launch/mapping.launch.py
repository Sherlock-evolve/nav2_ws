"""One-command bringup for mapping a Gazebo world with SLAM Toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('nav2_lab_bringup')
    launch_dir = os.path.join(bringup_share, 'launch')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gzclient = LaunchConfiguration('use_gzclient')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simple_room',
            description='World name under nav2_lab_worlds/worlds or absolute .world path.',
        ),
        DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model.'),
        DeclareLaunchArgument('x_pose', default_value='-1.2', description='Initial robot x position.'),
        DeclareLaunchArgument('y_pose', default_value='-1.2', description='Initial robot y position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'params', 'nav2_params.yaml'),
            description='Parameter file used by SLAM Toolbox.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('use_gzclient', default_value='true', description='Start Gazebo GUI.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={
                'world': world,
                'model': model,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw,
                'use_sim_time': use_sim_time,
                'use_gzclient': use_gzclient,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
        ),
    ])
