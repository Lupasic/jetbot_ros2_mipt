from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('robot_namespace')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_2',
        description='Robot namespace for topic scoping'
    )
    
    nav2_status_publisher_node = Node(
        package='jetbot_bringup',
        executable='nav2_status_publisher',
        name='nav2_status_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(nav2_status_publisher_node)
    
    return ld
