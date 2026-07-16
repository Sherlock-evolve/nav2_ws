import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('nav2_lab_bringup')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    enable_straight_line_planner = LaunchConfiguration('enable_straight_line_planner')
    enable_observe_spin = LaunchConfiguration('enable_observe_spin')
    enable_marker_layer = LaunchConfiguration('enable_marker_layer')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gzclient = LaunchConfiguration('use_gzclient')

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
                'use_gzclient': use_gzclient,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'bt_xml': bt_xml,
                'enable_straight_line_planner': enable_straight_line_planner,
                'enable_observe_spin': enable_observe_spin,
                'enable_marker_layer': enable_marker_layer,
                'slam': 'False',
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
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
            'bt_xml',
            default_value='',
            description=(
                'Optional NavigateToPose BT XML name under '
                'nav2_lab_bringup/behavior_trees or absolute .xml path.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_straight_line_planner',
            default_value='false',
            description='Use the custom StraightLinePlanner instead of the official Navfn planner.',
        ),
        DeclareLaunchArgument(
            'enable_observe_spin',
            default_value='false',
            description='Use the custom ObserveSpin behavior in the recovery tree.',
        ),
        DeclareLaunchArgument(
            'enable_marker_layer',
            default_value='false',
            description='Enable the custom MarkerLayer in the global costmap.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('use_gzclient', default_value='true', description='Start Gazebo GUI.'),
        OpaqueFunction(function=launch_setup),
    ])
