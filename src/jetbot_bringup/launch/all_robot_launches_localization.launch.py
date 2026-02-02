from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for robot_id
    declare_robot_id_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_2',
        description='ID of the robot, which is used as namespace.'
    )

    robot_namespace = LaunchConfiguration('robot_namespace')

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

    # Include navig.launch.py - will start 10 seconds after activate_all_drivers
    navig_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        jetbot_bringup_pkg_share,
                        'launch',
                        'localization.launch.py'
                    ])
                ),
                launch_arguments={'robot_namespace': robot_namespace}.items()
            )
        ]
    )

    # Robot navigation bridge - starts 20 seconds after activate_all_drivers (after Nav2)
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
    ld.add_action(activate_all_drivers_launch)
    ld.add_action(navig_launch)
    ld.add_action(robot_nav_bridge_launch)

    return ld
