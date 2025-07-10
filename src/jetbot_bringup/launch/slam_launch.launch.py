import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="robot_2",
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    robot_namespace = LaunchConfiguration("robot_namespace")

    slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={'<robot_namespace>': (robot_namespace, '/')},
    )

    slam_params_file = ParameterFile(
        RewrittenYaml(
            source_file=slam_params_file,
            root_key=robot_namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            slam_params_file
        ],
        remappings=[('/map','map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription(declared_arguments + [
        slam_toolbox,
    ])
