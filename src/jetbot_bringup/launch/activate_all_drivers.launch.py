from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Lidar launch include (если rplidar.launch.py лежит в этом же пакете)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "launch", "rplidar.launch.py"
            ])
        )
    )

    # Diffbot launch include из diffdrive_jetbot
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffdrive_jetbot"), "launch", "diffbot.launch.py"
            ])
        )
    )

    return LaunchDescription([
        lidar_launch,
        diffbot_launch,
    ])