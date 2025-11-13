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
    
    amcl_pose_relay_node = Node(
        package='jetbot_bringup',
        executable='amcl_pose_relay',
        name='amcl_pose_relay',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(amcl_pose_relay_node)
    
    return ld
