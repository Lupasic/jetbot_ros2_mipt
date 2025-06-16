from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_include = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diffbot_base_controller/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': PathJoinSubstitution([
                        FindPackageShare("jetbot_bringup"), 
                        "config", 
                        "nav2_params.yaml"
                    ]),
                }.items()
            )
        ]
    )
    return LaunchDescription([nav2_include])