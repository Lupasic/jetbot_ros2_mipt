from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ),
        launch_arguments=[
            ("slam_params_file", PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "slam_toolbox_online_async.yaml"
            ])),
        ]
    )
    return LaunchDescription([slam_launch])