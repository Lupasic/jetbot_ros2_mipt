import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_id",
            default_value="2",
            description="Unique robot ID (1-5)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "slam_toolbox_online_async.yaml"
            ]),
            description="Path to the slam parameters file"
        )
    )

    # Initialize Arguments
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    robot_namespace = [TextSubstitution(text='robot_'), robot_id]

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'base_frame': [TextSubstitution(text='robot_'), robot_id, TextSubstitution(text='/base_footprint')]},
            {'odom_frame': [TextSubstitution(text='robot_'), robot_id, TextSubstitution(text='/odom')]},
            {'map_frame': [TextSubstitution(text='robot_'), robot_id, TextSubstitution(text='/map')]},
            {'scan_topic': 'scan'}  # Will be namespaced automatically
        ],
        remappings=[('/map','map')
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription(declared_arguments + [
        slam_toolbox,
    ])
