from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _distributed_agent(context, *args, **kwargs):
    """Start the per-robot decentralized LCMAPFAgent only when distributed:=true."""
    distributed = LaunchConfiguration('distributed').perform(context).lower() \
        in ('true', '1', 'yes')
    if not distributed:
        return []
    ns = LaunchConfiguration('robot_namespace').perform(context)  # e.g. "robot_2"
    robot_id = int(ns.split('_')[-1])
    device = LaunchConfiguration('device').perform(context)
    # Delay so SLAM/TF are up before the agent reports cells in real-motion mode.
    return [TimerAction(period=20.0, actions=[Node(
        package='jetbot_bringup',
        executable='lcmapf_agent_node',
        name='lcmapf_agent',
        namespace=ns,
        parameters=[{'robot_id': robot_id, 'device': device}],
        output='screen',
        emulate_tty=True,
    )])]


def generate_launch_description():
    # Declare the launch argument for robot_id
    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_2',
        description='ID of the robot, which is used as namespace.'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jetbot_bringup'),
            'config',
            'nav2_default_localization_ignore_dyn_obst_test.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for navigation nodes'
    )

    declare_map_file_name_cmd = DeclareLaunchArgument(
        'map_file_name',
        default_value=PathJoinSubstitution([
            FindPackageShare('jetbot_bringup'),
            'maps',
            'map_labirint_v3'
        ]),
        description='Full path to the map file (without extension) for SLAM localization'
    )

    declare_distributed_cmd = DeclareLaunchArgument(
        'distributed',
        default_value='false',
        description='Start the decentralized LCMAPFAgent node for distributed inference.'
    )

    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='PyTorch device for the on-robot LCMAPFAgent (cpu recommended on Jetson).'
    )

    robot_namespace = LaunchConfiguration('robot_namespace')
    params_file = LaunchConfiguration('params_file')
    map_file_name = LaunchConfiguration('map_file_name')

    # Get the package share directory
    jetbot_bringup_pkg_share = FindPackageShare('jetbot_bringup')
    
    # Include activate_all_drivers.launch.py
    # Assuming it is in the same package and accepts a 'namespace' argument.
    activate_all_drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                jetbot_bringup_pkg_share,
                'launch',
                'activate_all_drivers.launch.py'
            ])
        ),
        launch_arguments={'robot_namespace': robot_namespace}.items()
    )

    # Include slam_localization.launch.py - will start 10 seconds after activate_all_drivers
    slam_localization_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        jetbot_bringup_pkg_share,
                        'launch',
                        'slam_localization.launch.py'
                    ])
                ),
                launch_arguments={
                    'robot_namespace': robot_namespace,
                    'map_file_name': map_file_name
                }.items()
            )
        ]
    )

    # Include navig.launch.py - will start 15 seconds after activate_all_drivers (same time as SLAM)
    navig_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        jetbot_bringup_pkg_share,
                        'launch',
                        'navig.launch.py'
                    ])
                ),
                launch_arguments={
                    'robot_namespace': robot_namespace,
                    'params_file': params_file
                }.items()
            )
        ]
    )

    # Robot navigation bridge - starts 20 seconds after activate_all_drivers (after SLAM and Nav2)
    robot_nav_bridge_launch = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='jetbot_bringup',
                executable='robot_nav_bridge',
                name='robot_nav_bridge',
                namespace=robot_namespace,
                parameters=[{
                    'robot_namespace': robot_namespace
                }],
                output='screen'
            )
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_robot_id_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_file_name_cmd)
    ld.add_action(declare_distributed_cmd)
    ld.add_action(declare_device_cmd)
    ld.add_action(activate_all_drivers_launch)
    ld.add_action(slam_localization_launch)
    ld.add_action(navig_launch)
    ld.add_action(robot_nav_bridge_launch)
    ld.add_action(OpaqueFunction(function=_distributed_agent))

    return ld
