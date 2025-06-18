from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    twist_stamped_to_twist = Node(
        package='topic_tools',
        executable='relay_field',
        name='twist_stamped_to_twist',
        arguments=[
            '/cmd_vel_robot_steering_stamped',      # input topic
            '/cmd_vel_robot_steering',              # output topic
            'geometry_msgs/msg/Twist',              # output type
            '{linear: m.twist.linear, angular: m.twist.angular}'                               # expression on m (m — входящее сообщение)
        ]
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[PathJoinSubstitution([
                FindPackageShare("jetbot_bringup"), "config", "twist_mux.yaml"
            ])],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

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
        twist_stamped_to_twist,
        twist_mux,
        lidar_launch,
        diffbot_launch,
    ])